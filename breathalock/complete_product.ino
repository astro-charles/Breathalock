#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BreathalockConfig.h"

#include <Adafruit_Fingerprint.h>

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


SoftwareSerial mySerial(FINGER_SOFTWARE_SERIAL_TX, FINGER_SOFTWARE_SERIAL_RX);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
boolean fingerprint_unlocked = false;
//boolean bleEnabled = false;
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
   pinMode(MOSTFET_GAS_HEATER, OUTPUT);
   pinMode(MOSFET_FINGERPRINT,OUTPUT);
   pinMode(RED_RGB_PIN_STATUS, OUTPUT);
   pinMode(BLUE_RGB_PIN_STATUS, OUTPUT);
   pinMode(GREEN_RGB_PIN_STATUS, OUTPUT);
   digitalWrite(MOSFET_FINGERPRINT,HIGH); //This is for unlocking the MOS
   
   digitalWrite(MOSTFET_GAS_HEATER,LOW); //This is for unlocking the MOS
   if(!initBLEDevice()) {
      Serial.println("Initialised Bluefruit LE module...");
   }
   
   ble.end(); // Why is this here?
   finger.begin(57600);
   
   if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
   } else {
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
  char commandToSend[25];
  String command= "AT+BLEUARTTX= ";
  int alcoholRead = readAlcoholLevel();

  String readToStr = String(alcoholRead);

  readToStr.concat("\n");
  command.concat(readToStr);
  command.toCharArray(commandToSend,25);
  
  
  if(fingerprint_unlocked){
    //if(ble.isConnected()) {
      Serial.println(commandToSend);
      ble.sendCommandCheckOK(commandToSend);
      //delay(500);
    //}
  }else {
    colorChange(255,0,0);
    if(getFingerprintStatus() != -1) {
      colorChange(0,255,0);
      Serial.println("fingerprint found");
      fingerprint_unlocked = true;
      finger.end();
      digitalWrite(MOSFET_FINGERPRINT,LOW); //This is for unlocking the MOS
      delay(500);
      digitalWrite(MOSTFET_GAS_HEATER,HIGH); //This is for unlocking the MOS
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

int readAlcoholLevel() {
  int mq3Average          = 0;
//  int mq3value_readBase   = 0;
  int mq3value_readOne    = 0;
  int mq3value_readTwo    = 0;
  int mq3value_readThree  = 0;
  int mq3value_readFour   = 0;
  
    
  mq3value_readOne   = analogRead(GAS_SENSOR_PIN);
  delay(50);
  mq3value_readTwo   = analogRead(GAS_SENSOR_PIN);
  delay(50);
  mq3value_readThree = analogRead(GAS_SENSOR_PIN);
  delay(50);
  mq3value_readFour  = analogRead(GAS_SENSOR_PIN);
  
  mq3Average = (mq3value_readOne +
                mq3value_readTwo +
                mq3value_readThree +
                mq3value_readFour);
  mq3Average /= 4;
  return (int) mq3Average;
 
} 

/**************************************************************************/
/*!
    @brief     read the gas sensor value
    @return    return the average value take from the gas sensor
    @see       https://cdn.sparkfun.com/datasheets/Sensors/Biometric/MQ-3%20ver1.3%20-%20Manual.pdf
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
//  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID; 
}
