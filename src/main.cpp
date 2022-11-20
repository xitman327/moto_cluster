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
#define SD_DET_DELAY 10
bool sd_ok, sd_err;

#include <mcp2515.h>
MCP2515 CAN(PA4, 10000000UL, &SPI);
#define CAN0_INT PB5

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


bool can_ok, mpu_ok, hmc_ok, tmp_ok, tft_ok;
void setup() {

  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  //init first for debugging
  BTSERAIL.begin(115200);
  BTSERAIL.println("starting...........");
  BTSERAIL.println("BTSERAIL ok");

  //init rtc
  rtc.begin(false, STM32RTC::HOUR_24);
  BTSERAIL.println("RTC ok");

  //use seccond SPI module
  // SPI.setMISO(PB14);
  // SPI.setMOSI(PB15);
  // SPI.setSCLK(PB13);
  // SPI.begin();

  //initialize canbus module
  uint8_t y;// = CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
  y = CAN.reset();
  if(!y){
    can_ok = 1;
    BTSERAIL.println("can ok");
    CAN.setBitrate(CAN_500KBPS);
    CAN.setNormalMode();
  }else{
    BTSERAIL.print("can failled ");
    BTSERAIL.println(y);
  }

  //init LCD
  tft.begin();
  uint8_t x = tft.readcommand8(ILI9488_RDMODE);
  if(x == 0xff || x == 0x00){
    BTSERAIL.println("tft failled");
    tft_ok = 0;
  }else{
    BTSERAIL.println("tft ok");
    tft_ok = 1;
    tft.setRotation(0);
  }
  

  //init touch
  touch.begin(tft.width(), tft.height());
  touch.setRotation(touch.ROT0);
  touch.setCalibration(209, 1759, 1775, 273);//we need to polish this, also create a calibration sceme
  BTSERAIL.println("touch ok");

  //we do that later
  // if (!SD.begin(SD_CS)) {

  // }

  //init neopixels
  pixels.begin();
  BTSERAIL.println("neopixel ok");

  //serial for gps
  GPSSERIAL.begin(9600);
  BTSERAIL.println("gps ok");

  //serial for bluetooth
  //BTSERAIL.begin(115200);

  //i2c for sensors
  Wire.setSCL(PB10);
  Wire.setSDA(PB11);

  //initialize mpu
  mpu_ok = accelgyro.testConnection();
  if(mpu_ok){
    accelgyro.initialize();
    BTSERAIL.println("MPU ok");
  }else{
    BTSERAIL.println("MPU failled");
  }
    

  // initialize compass
  hmc_ok = compass.begin();
  if(hmc_ok){
    BTSERAIL.println("HMC ok");
  }else{
    BTSERAIL.println("HMC failled");
  }

  //initialize temp/humid sensor
  tmp_ok = aht10.begin();
  if(tmp_ok){
    BTSERAIL.println("TMP ok");
  }else{
    BTSERAIL.println("TMP failled");
  }

  BTSERAIL.println("END SETUP");

}

void loop() {

}

void handle_gps(){

  if(GPSSERIAL.available()){
    valid_gps = gps.encode(GPSSERIAL.read());
  }

}

void handle_sd(){

  if(!digitalRead(SD_DET) && !sd_ok && !sd_err){
    delay(SD_DET_DELAY);
    if(!SD.begin(SD_CS)){
      sd_ok = 0;
      sd_err = 1;
    }else{
      sd_ok = 1;
    }
  }else if(digitalRead(SD_DET) && (sd_ok || sd_err)){
    SD.end();
    sd_ok = 0;
    sd_err = 0;
  }

}