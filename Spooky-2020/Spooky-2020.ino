/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

const int WHITE_BUTTON_INPUT = 6;
const int GREEN_BUTTON_INPUT = 12;
const int YELLOW_BUTTON_INPUT = 13;

const int WHITE_BUTTON_LED = 5;
const int GREEN_BUTTON_LED = 9;
const int YELLOW_BUTTON_LED = 10;

const bool ENABLE_LOCAL_SERIAL = false;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  delay(500);

  if(ENABLE_LOCAL_SERIAL){
    Serial.begin(115200);
    Serial.println(F("Adafruit Bluefruit Command Mode Example"));
    Serial.println(F("---------------------------------------"));
  }

  /* Initialise the module */
  serialPrint(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  serialPrintLn( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    serialPrintLn(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  serialPrintLn("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  serialPrintLn(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  serialPrintLn(F("Then Enter characters to send to Bluefruit"));
  serialPrintLn("");

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    serialPrintLn(F("******************************"));
    serialPrintLn(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    serialPrintLn(F("******************************"));
  }
  
  //Setup Buttons
  pinMode(WHITE_BUTTON_INPUT,INPUT_PULLUP);
  pinMode(GREEN_BUTTON_INPUT,INPUT_PULLUP);
  pinMode(YELLOW_BUTTON_INPUT,INPUT_PULLUP);
  
  //Setup LEDs
  pinMode(WHITE_BUTTON_LED,OUTPUT);
  pinMode(GREEN_BUTTON_LED,OUTPUT);
  pinMode(YELLOW_BUTTON_LED,OUTPUT);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for user input
  char inputs[BUFSIZE+1];
  
  checkButton(WHITE_BUTTON_INPUT, "WHITE");
  checkButton(GREEN_BUTTON_INPUT, "GREEN");
  checkButton(YELLOW_BUTTON_INPUT, "YELLOW");
  
  digitalWrite(WHITE_BUTTON_LED, HIGH);
  digitalWrite(GREEN_BUTTON_LED, HIGH);
  digitalWrite(YELLOW_BUTTON_LED, HIGH);


  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    serialPrint("[Send] ");
    serialPrintLn(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      serialPrintLn(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, its in the buffer
  serialPrint(F("[Recv] ")); 
  serialPrintLn(ble.buffer);
  ble.waitForOK();
}

void checkButton(int button, String color){
  int btnVal = digitalRead(button);

  if(btnVal == LOW){
    delay(5);
    btnVal = digitalRead(button);
    if(btnVal == LOW){
      serialPrintLn(color + "BUTTON DOWN \r ");
      ble.print("AT+BLEUARTTX=");
      ble.print(" -- " + color + " BUTTON DOWN -- \n \n \r\n \n\r "); 
      delay(1000);
    }
  }
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  if(!ENABLE_LOCAL_SERIAL){
    return false;
  }
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}

void serialPrintLn(String s){
  if(ENABLE_LOCAL_SERIAL){
    Serial.println(s);
  }
}

void serialPrint(String s){
  if(ENABLE_LOCAL_SERIAL){
    Serial.print(s);
  }
}
