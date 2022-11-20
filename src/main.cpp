#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <STM32RTC.h>
STM32RTC& rtc = STM32RTC::getInstance();

#include <Adafruit_GFX.h>
#include <ILI9488.h>
#define TFT_CS         PB12
#define TFT_DC         PA15
#define TFT_LED        PA8
#define TFT_RST        -1
ILI9488 tft = ILI9488(TFT_CS, TFT_DC, TFT_RST);

#include <XPT2046.h>
XPT2046 touch(/*cs=*/ PB3, /*irq=*/ PB4);

#include <SD.h>
#define SD_CS PB7
#define SD_DET PB8
File myFile;

#include <mcp_can.h>
#define CAN0_INT PB5                              /* Set INT to pin 2 (This rarely changes)   */
MCP_CAN CAN0(PB6);                                /* Set CS to pin 9 (Old shields use pin 10) */

#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN PB9
#define NUMPIXELS 15 
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

#include <TinyGPS.h>
TinyGPS gps;
bool valid_gps;
HardwareSerial GPSSERIAL(PA10, PA9);

#include <MPU6050.h>
MPU6050 accelgyro;  

#include <HMC5883L.h>
HMC5883L compass;

#include <AHTxx.h>
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR);

HardwareSerial BTSERAIL(PA3, PA2);

#define LIGHT_SENSOR PA1
#define INPUT1 PA4
#define INPUT2 PA5
#define INPUT3 PA6
#define INPUT4 PA7

void setup() {

  //init rtc
  rtc.begin(false, STM32RTC::HOUR_24);

  //use seccond SPI module
  SPI.setMISO(PB14);
  SPI.setMOSI(PB15);
  SPI.setSCLK(PB13);
  SPI.setSSEL(-1);

  //init LCD
  tft.begin();
  tft.setRotation(0);

  //init touch
  touch.begin(tft.width(), tft.height());
  touch.setRotation(touch.ROT0);
  touch.setCalibration(209, 1759, 1775, 273);//we need to polish this, also create a calibration sceme

  //initialize canbus module
  if(!CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK){
    
  }

  //we do that later
  // if (!SD.begin(SD_CS)) {

  // }

  //init neopixels
  pixels.begin();

  //serial for gps
  GPSSERIAL.begin(9600);

  //serial for bluetooth
  BTSERAIL.begin(115200);

  //i2c for sensors
  Wire.setSCL(PB10);
  Wire.setSDA(PB11);

  //initialize mpu
  accelgyro.initialize();

  // initialize compass
  if(!compass.begin()){

  }

  //initialize temp/humid sensor
  if(!aht10.begin()){

  }


}

void loop() {

}

void handle_gps(){
  if(GPSSERIAL.available()){
    valid_gps = gps.encode(GPSSERIAL.read());
  }
}