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

// This value is used for determining if the MQ3 has been intilized yet
boolean bMQ3Initialized = false;

// The device uses this value to determine if the countdown timer needs to be activated again
// this is only used once through the entirity of the programs life.
float fBaselineAlcoholValue = 0.00;

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
   
   
   if(!initBLEDevice()) {
      Serial.println("Initialised Bluetooth LE module...");
   }
  
   finger.begin(57600);
   Serial.print("Initialising Fingerprint module: ");
   if (!finger.verifyPassword()) {
    Serial.println("Did not find fingerprint sensor");
    bUnlockedDeviceFingerprint = true;
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
    colorChange(0,255,0); // green
     
    /* stringify our float value then concat it with our bluetooth ATI command  */
    alcoholRead = readAlcoholLevel(100);
    stringifyAlcohol(alcoholRead).toCharArray(commandToSend,24);
    //DEBUG
    Serial.print("alcoholValue = "); Serial.println(commandToSend);
    isOverLimit(alcoholRead,fBaselineAlcoholValue);
    /* send stringified command (XXX is gas value): AT+BLEUARTTX= XXX */
    ble.sendCommandCheckOK(commandToSend);
  }else {
    /* notify the users without smart phones that the device is locked */
    colorChange(255,0,0); // red
    if(getFingerprintStatus() != -1) {
      /* finger print was found unlock the rest of the devices functionality */
      bUnlockedDeviceFingerprint = true;
      finger.end(); // end all fingerprint activity until reset (physical shutoff or push button reset
      
      //TODO: evaluate if delay is needed
      delay(500); 
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
  /* BEGIN DEBUG */
  Serial.print("baseline "); Serial.println(baseline_alcValue);
  Serial.print("baseline*multiplier: "); Serial.println(baseline_alcValue*(PERCENTAGE_TOLERANCE/100));
  /* END DEBUG */
  
  if (alcValue >= baseline_alcValue*(PERCENTAGE_TOLERANCE/100)) {
    digitalWrite(GATE_POWER_PIN,LOW);
    digitalWrite(STATUS_LIGHT_YELLOW,HIGH);
    return true;
  }
  //TODO: Identify how to tell if the user even blew at all.
  digitalWrite(GATE_POWER_PIN,HIGH);
  digitalWrite(STATUS_LIGHT_YELLOW,LOW);
  return false;
}


/**************************************************************************/
/*!
    @brief     this is how we check if the user has blown into the device if not remain car locked
    @return    return true if user has blown
*/
/**************************************************************************/
//TODO: most important otherwise users bypass alchol sensor everytime by not blowing
// @assign nick and nam help!
boolean isUserBlowing(float value) {
  
  return false;
}

/**************************************************************************/
/*!
    @brief     change status light led to corresponding color
    @return    return true on success
*/
/**************************************************************************/
boolean colorChange(int red, int blue, int green) {
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
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;
  
  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID); 
  //To identify confidence use finger.confidence;however, practically useless.
  return finger.fingerID; 
}
