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


boolean fingerprint_unlocked = false;
boolean first_time_reading = true;
float baseline_alcohol_val = 0;

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
   pinMode(GATE_POWER_PIN, OUTPUT);
   pinMode(RED_RGB_PIN_STATUS, OUTPUT);
   pinMode(BLUE_RGB_PIN_STATUS, OUTPUT);
   pinMode(GREEN_RGB_PIN_STATUS, OUTPUT);
   pinMode(STATUS_LIGHT_YELLOW, OUTPUT);
   
   if(!initBLEDevice()) {
      Serial.println("Initialised Bluefruit LE module...");
   }
  
   finger.begin(57600);
   
   if (!finger.verifyPassword()) {
    Serial.println("Did not find fingerprint sensor");
    fingerprint_unlocked = true;
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
  String ATICommand= "AT+BLEUARTTX= ";
  float alcoholRead = 0;
  String alcReadToStr;
  unsigned long currentMillis = millis();
  
  if(first_time_reading && currentMillis >= WARMUP_TIME_MS) {
    first_time_reading = false;
    baseline_alcohol_val = readAlcoholLevel(100);
    Serial.println("first read complete value is: ");
    Serial.println(baseline_alcohol_val);
  }
  
  if(fingerprint_unlocked && !first_time_reading){
    colorChange(0,255,0); // green
    /* stringify our float value then concat it with our bluetooth ATI command  */
    alcoholRead = readAlcoholLevel(100);
    alcReadToStr = String(alcoholRead);
    alcReadToStr.concat(" ");
    ATICommand.concat(alcReadToStr);
    ATICommand.toCharArray(commandToSend,45);
    /* end block stringification */ 
    
    //DEBUG
    Serial.print("alcholValue = "); Serial.println(alcReadToStr);
    
    /* send stringified command (XXX is gas value): AT+BLEUARTTX= XXX */
    ble.sendCommandCheckOK(commandToSend);
    
//    if(alcoholRead >= 4.00) {
//      digitalWrite(STATUS_LIGHT_YELLOW,HIGH);
//      digitalWrite(GATE_POWER_PIN,LOW);
//    }else {
//      digitalWrite(STATUS_LIGHT_YELLOW,LOW);
//      digitalWrite(GATE_POWER_PIN,HIGH);
//    }
  }else {
    /* notify the users without smart phones that the device is locked */
    colorChange(255,0,0); // red
    if(getFingerprintStatus() != -1) {
      /* finger print was found unlock the rest of the devices functionality */
      fingerprint_unlocked = true;
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
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in CMD mode & check wiring?"));
    return false;
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  
  ble.info();

  ble.verbose(VERBOSE_MODE); 
  ble.sendCommandCheckOK("AT+GAPDEVNAME=" DEVICE_NAME);
  
  return true;
}

/**************************************************************************/
/*!
    @brief     this is how we will check the users limit against our delta defined in our header file.
    @return    return true if user is over limit
*/
/**************************************************************************/
boolean isOverLimit(float value) {

  return false;
}


/**************************************************************************/
/*!
    @brief     this is how we check if the user has blown into the device if not remain car locked
    @return    return true if user has blown
*/
/**************************************************************************/
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
