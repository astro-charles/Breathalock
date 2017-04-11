#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BreathalockConfig.hpp"
#include <Adafruit_Fingerprint.h>

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

SoftwareSerial mySerial(FINGER_SOFTWARE_SERIAL_TX, FINGER_SOFTWARE_SERIAL_RX);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// The device has been unlocked via the fingerprint sensor this needs 
// be global so that the device can remember the device has been unlocked 
boolean bUnlockedDeviceFingerprint = false;
boolean bFingerprintFailed = false;
// This value is used for determining if the MQ3 has been intilized yet
boolean bMQ3Initialized = false;
//This boolean value is for determining if the user has blown or not.
boolean bUserHasBlown = false;
//
boolean bUserFailedToAuth = false;
// The device uses this value to determine if the countdown timer needs to be activated again
// this is only used once through the entirity of the programs life.
float fBaselineAlcoholValue = 0.00;
int failed_attempts=0;

boolean bIsUserOverTheLimit = false;


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
   Serial.begin(9600);
   delay(500);
   pinMode(GATE_POWER_PIN, OUTPUT); // for the unlock button
   pinMode(STATUS_LIGHT_YELLOW, OUTPUT); // if drunk this comes on
   pinMode(RED_RGB_PIN_STATUS, OUTPUT);
   pinMode(BLUE_RGB_PIN_STATUS, OUTPUT);
   pinMode(GREEN_RGB_PIN_STATUS, OUTPUT);
   digitalWrite(STATUS_LIGHT_YELLOW,HIGH);
   
   if(!initBLEDevice()) {
      Serial.println("Initialised Bluetooth LE module...");
   }
  
   finger.begin(57600);
   Serial.print("Initialising Fingerprint module: ");
   if (!finger.verifyPassword()) {
    Serial.println("Did not find fingerprint sensor");
    //Fingerprint not found allow the user to proceed with full functionality see comment below
    bUnlockedDeviceFingerprint = true;
    //The fingerprint device is not found disable unlocking ability
    bFingerprintFailed=true;
   }else {
    Serial.println("OK!");
   }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  char commandToSend[24];
  float alcoholRead = 0;
  
  if(!bMQ3Initialized) { // Check if MQ3 has been initialized yet
    fBaselineAlcoholValue = recordBaseline();
  }
  if(bUnlockedDeviceFingerprint && bMQ3Initialized){
//    if(bFingerprintFailed) {
//      colorChange(0,0,255); // blue
//    }else {
//      colorChange(0,255,0); // green
//    }
    if(bUserHasBlown) {
    /* stringify our float value then concat it with our bluetooth ATI command  */
      alcoholRead = readAlcoholLevel(100);
      stringifyAlcohol(alcoholRead).toCharArray(commandToSend,24);
      //BEGIN DEBUG
      Serial.print("alcoholValue = "); Serial.println(commandToSend);
      //END DEBUG
      if(!bIsUserOverTheLimit) {
        bIsUserOverTheLimit = isOverLimit(alcoholRead,fBaselineAlcoholValue);
      }
      
      /* send stringified command (XXX is gas value): AT+BLEUARTTX= XXX */
      ble.sendCommandCheckOK(commandToSend);
    }else {
      bUserHasBlown = isUserBlowing();
      Serial.println("testing blown");
    }
  }else {
    /* notify the users without smart phones that the device is locked */
    if(!bUserFailedToAuth) {
      if(getFingerprintStatus() != -1) {
        /* finger print was found unlock the rest of the devices functionality */
        bUnlockedDeviceFingerprint = true;
        finger.end(); // end all fingerprint activity until reset (physical shutoff or push button reset
        
        //TODO: evaluate if delay is needed
        delay(250); 
      } 
    }
  }
}

/**************************************************************************/
/*!
    @brief  initialize the bluetooth device
    @return true if successful otherwise return false
*/
/**************************************************************************/
bool initBLEDevice() {
  /* Initialise the module */
  Serial.print(F("Initialising Bluetooth LE module:: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in CMD mode & check wiring?"));
    return false;
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.verbose(VERBOSE_MODE); 
  ble.sendCommandCheckOK("AT+GAPDEVNAME=" DEVICE_NAME);
  
  return true;
}
/**************************************************************************/
/*!
    @brief     convert alcohol Value to string
    @return    return false after 15secs
*/
/**************************************************************************/
String stringifyAlcohol(float alcohol_val) {
  String ATICommand= "AT+BLEUARTTX= ";
  String alcReadToStr;
  
  alcReadToStr = String(alcohol_val);
  alcReadToStr.concat(" ");
  ATICommand.concat(alcReadToStr);

  return ATICommand;
}

/**************************************************************************/
/*!
    @brief     This gives us our inital warmed up value that we utilize in comparisons later
    @return    return false after 15secs
*/
/**************************************************************************/
float recordBaseline() {
  if(millis() >= WARMUP_TIME_MS && !bMQ3Initialized) {
    float bl_alcohol_val = readAlcoholLevel(1000);
    /* BEGIN DEBUG */ 
    Serial.println("Initialised MQ3 module...");
    Serial.print("Baseline Alcohol Recorded at : "); Serial.println(bl_alcohol_val);
    /* END DEBUG */ 
    bMQ3Initialized = true;
    digitalWrite(STATUS_LIGHT_YELLOW,LOW);
    return bl_alcohol_val;
  } 
  return 0.00;
}
/**************************************************************************/
/*!
    @brief     this is how we will check the users limit against our delta defined in our header file.
    @return    return true if user is over limit
*/
/**************************************************************************/

boolean isOverLimit(float alcValue, float baseline_alcValue) {
  boolean userIsOverLimit = false;
  float delta = 0;
  /* BEGIN DEBUG */
  Serial.print("isOverlimit: alcValue= ");
  Serial.println(alcValue);
  /* END DEBUG */

  //determine if user is over the limit
  delta = getDeltaOfBlow();
  //if the user is over the tolerance_delta value and the delta is in the positive direction fail the user.
  if(delta > ALCOHOL_TOLERANCE_DELTA && delta > 0) {
    userIsOverLimit = true;
  }
  
  if(userIsOverLimit) {
    digitalWrite(GATE_POWER_PIN,LOW);
    colorChange(50,0,0);
    return true;
  }else {
    if(!bFingerprintFailed) {
      digitalWrite(GATE_POWER_PIN,HIGH); 
    }
    colorChange(0,50,0); //green
    return false;
  }
}

float getDeltaOfBlow() {
  float mq3FirstRead = 0;
  float mq3SecondRead = 0;
  float delta = 0;
  
  mq3FirstRead = readAlcoholLevel(100);
  delay(500);
  mq3SecondRead = readAlcoholLevel(100);
 
  delta =  mq3SecondRead - mq3FirstRead;
  return delta;
}
/**************************************************************************/
/*!
    @brief     this is how we check if the user has blown into the device if not remain car locked
    @return    return true if user has blown
*/
/**************************************************************************/
//TODO: most important otherwise users bypass alchol sensor everytime by not blowing
// @assign nick and nam help!
boolean isUserBlowing() {
  float mq3Value1 = 0;
  float mq3Value2 = 0;
  float actualDelta = 0;
  
  mq3Value1 = readAlcoholLevel(100);
  delay(500);
  mq3Value2 = readAlcoholLevel(100);
 
  actualDelta =  abs(mq3Value2*100 - mq3Value1*100);
  //BEGIN DEBUG
  Serial.println("");
  Serial.print("VALUE 1: "); Serial.println(mq3Value1*100);
  Serial.print("VALUE 2: "); Serial.println(mq3Value2*100);
  Serial.print("DELTA: ");   Serial.println(actualDelta);
  Serial.println("");
  //END DEBUG
  if (actualDelta*100 > BREATH_DELTA*100) {
    return true;
  }
  return false;
}

/**************************************************************************/
/*!
    @brief     change status light led to corresponding color
    @return    return true on success
*/
/**************************************************************************/
boolean colorChange(int red, int blue, int green ) {
  
  analogWrite(RED_RGB_PIN_STATUS,red);
  analogWrite(GREEN_RGB_PIN_STATUS,green);
  analogWrite(BLUE_RGB_PIN_STATUS,blue);
  
  return true;
}

/**************************************************************************/
/*!
    @brief     read the gas sensor value
    @return    return the average value take from the gas sensor
    @see       https://cdn.sparkfun.com/datasheets/Sensors/Biometric/MQ-3%20ver1.3%20-%20Manual.pdf
*/
/**************************************************************************/

float readAlcoholLevel(int reads) {
  float mq3Sum=0;
  float mq3RealValue = 0;
  float mq3Average = 0;
  
  //sum together READs number of analogReads for the gas sensor to collect an avg
  for(int i=0;i<reads;i++) {
    mq3Sum = mq3Sum + analogRead(GAS_SENSOR_PIN);
  }
  
  mq3Average = mq3Sum/reads;
  mq3RealValue = (mq3Average/1024)*5.0;
  
  return mq3RealValue;
} 

/**************************************************************************/
/*!
    @brief     get the fingerprint statu
    @return    return the the id found that matches the fingerprint stored on the sensor itself
    @see       https://www.adafruit.com/product/751
*/
/**************************************************************************/
int getFingerprintStatus() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  {
    return -1;
  }

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK) {
    return -1;
  }

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  {
    Serial.println(failed_attempts);
    if(failed_attempts >= (NUMBER_OF_FAILED_ATTEMPTS-1)) {
      colorChange(80,25,0); // yellow on failed finger scan
      finger.end();
      bUserFailedToAuth = true;
      
    }
    failed_attempts++;
    return -1;
  }
  
  // found a match!
  colorChange(0, 0, 50); // green on pass
  Serial.print("Found ID #"); Serial.print(finger.fingerID); 
  //To identify confidence use finger.confidence;however, practically useless.
  return finger.fingerID; 
}
