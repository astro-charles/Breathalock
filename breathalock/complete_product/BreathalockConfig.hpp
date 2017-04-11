// COMMON SETTINGS

//GAS SENSOR SETTINGS
#define PERCENTAGE_TOLERANCE            30 //
#define V_LOAD_RESISTOR                 10 // kOhms
#define WARMUP_TIME_MS                  15000  // milliseconds    
#define BREATH_DELTA                    0.05    
#define ALCOHOL_TOLERANCE_DELTA         0.50         

// NAME HERE SETTINGS
// ----------------------------------------------------------------------------------------------
// Description here
// ----------------------------------------------------------------------------------------------
#define GATE_POWER_PIN                       4    
#define GAS_SENSOR_PIN                       A0 
// RGB STATUS LIGHTS
#define RED_RGB_PIN_STATUS                   3
#define BLUE_RGB_PIN_STATUS                  6
#define GREEN_RGB_PIN_STATUS                 5
//FINGERPRINT
// pin #7 is IN from sensor (GREEN wire) 
// pin #2 is OUT from arduino  (WHITE wire)
#define FINGER_SOFTWARE_SERIAL_TX            7 
#define FINGER_SOFTWARE_SERIAL_RX            2 
#define NUMBER_OF_FAILED_ATTEMPTS            3
//Status Light
#define STATUS_LIGHT_YELLOW                  13

#define MOSFET_GAS_HEATER                    A2 
#define MOSFET_FINGERPRINT                   A1 
       

// SOFTWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// You should use this option if you are connecting the UART Friend to an UNO
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       9    // Required for softwa`re serial!
#define BLUEFRUIT_SWUART_TXD_PIN       10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         8   // Optional, set to -1 if unused

// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   false  // If set to 'true' enables debug output
#define DEVICE_NAME                    "Breathalock"

// HARDWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the HW serial port you are using. Uncomment
// this line if you are connecting the BLE to Leonardo/Micro or Flora
// ----------------------------------------------------------------------------------------------
#ifdef Serial    // this makes it not complain on compilation if there's no Serial1
  #define BLUEFRUIT_HWSERIAL_NAME      Serial
#endif


// SHARED UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following sets the optional Mode pin, its recommended but not required
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_UART_MODE_PIN        -1    // Set to -1 if unused




