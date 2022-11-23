#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// #include <STM32RTC.h>
// STM32RTC& rtc = STM32RTC::getInstance();

#include <Adafruit_GFX.h>
// #include <ILI9488.h>
#define TFT_CS         26
#define TFT_DC         27
#define TFT_LED        14
#define TFT_RST        -1
// ILI9488 tft = ILI9488(TFT_CS, TFT_DC, TFT_RST);
// #include <Adafruit_PCD8544.h>
// Adafruit_PCD8544 display = Adafruit_PCD8544(TFT_DC, TFT_CS, TFT_RST);

#include <XPT2046.h>
XPT2046 touch(/*cs=*/ 25, /*irq=*/ 2);

#include <FS.h>
#include <SD.h>
#define SD_CS 15
#define SD_DET 13
File myFile;
#define SD_DET_DELAY 10
bool sd_ok, sd_err;
const char* anim_filenames[10] PROGMEM={"/ANI/0.txt", "/ANI/1.txt", "/ANI/2.txt", "/ANI/3.txt", "/ANI/4.txt","/ANI/5.txt","/ANI/6.txt","/ANI/7.txt","/ANI/8.txt","/ANI/9.txt"};
uint8_t animNumber;

#include <ESP32CAN.h>
#include <CAN_config.h>
CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 10;       // Receive Queue size
CAN_frame_t rx_frame;
CAN_frame_t tx_frame;

uint8_t msgType, errorType;
bool canWaitingMsg;
#define can_refresh 1000
uint32_t can_tm;

int16_t engineRPM, engineTemp;
double ecuVoltage;


#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN 12
#define NUMPIXELS 15 
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#include <neoAnim.h>
uint32_t sdpixeldata[1000];
uint16_t sdAnimLength, sdAnimFPS, sdAnimBrightness;

// Global values used by the animation and sound functions
uint32_t         *pixelBaseAddr; // Address of active animation table
uint16_t          pixelLen,      // Number of pixels in active table
                  pixelIdx,      // Index of first pixel of current frame
                  audioLen;      // Number of audio samples in active table
volatile uint16_t audioIdx;      // Index of current audio sample
uint8_t           pixelFPS,      // Frames/second for active animation
                 *audioBaseAddr; // Address of active sound table
bool              pixelLoop,     // If true, animation repeats
                  audioLoop;     // If true, audio repeats


// #include <TinyGPS.h>
// TinyGPS gps;
#define GPSSERIAL Serial2
#include <NMEAGPS.h>
//#include <GPSport.h>
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
bool valid_gps;


#include <MPU6050.h>
MPU6050 mpu;  


#include <HMC5883L.h>
HMC5883L compass;


#include <AHTxx.h>
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR);



#define LIGHT_SENSOR 32
#define INPUT1 34
#define INPUT2 35
#define INPUT3 36
#define INPUT4 39


bool can_ok, mpu_ok, hmc_ok, tmp_ok, tft_ok;

uint8_t random_startup;

bool start_anim_finished;
byte anim_i;
uint32_t default_anim[255] = {};

void lighting();
void handle_gps();
void handle_sd();
void playAnim(const uint32_t *addr, uint8_t fps, uint16_t bytes, bool repeat);
void handle_can();
void handle_sensors();

void setup() {

  // __HAL_AFIO_REMAP_SWJ_NOJTAG();

  //init first for debugging
  Serial.begin(115200);
  Serial.println("starting...........");
  Serial.println("Serial ok");

  //init rtc
  // rtc.begin(false, STM32RTC::HOUR_24);
  // Serial.println("RTC ok");

  //use seccond SPI module
  // SPI.setMISO(PB14);
  // SPI.setMOSI(PB15);
  // SPI.setSCLK(PB13);
  // SPI.setSSEL(PB12);
  // SPI.setClockDivider(SPI_CLOCK_DIV8);
  //SPI.begin();

  //initialize canbus module
  // pinMode(CAN0_INT, INPUT_PULLUP);
  // uint8_t y;// = CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
  // y = CAN.reset();
  // if(!y){
  //   can_ok = 1;
  //   Serial.println("can ok");
  //   CAN.setBitrate(CAN_500KBPS);
  //   CAN.setNormalMode();
  // }else{
  //   Serial.print("can failled ");
  //   Serial.println(y);
  // }

  // outMSG.can_id = 0x7DF;
  // outMSG.can_dlc = 8;
  // y = CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  // if(y == CAN_OK){
  //   Serial.println("can ok");
  // }else{
  //   Serial.print("can failled ");
  //   Serial.println(y);
  // }

  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
  tx_frame.FIR.U = 0x8;
  tx_frame.MsgID = 0x7DF;

  //init LCD
  // tft.begin();
  // uint8_t x = tft.readcommand8(ILI9488_RDMODE);
  // if(x == 0xff || x == 0x00){
  //   Serial.println("tft failled");
  //   tft_ok = 0;
  // }else{
  //   Serial.println("tft ok");
  //   tft_ok = 1;
  //   tft.setRotation(0);
  // }
  // display.begin();
  // display.clearDisplay();
  // display.print("hello world!");
  // Serial.println("lcd ok");
  

  //init touch
  touch.begin(480, 320);
  touch.setRotation(touch.ROT0);
  touch.setCalibration(209, 1759, 1775, 273);//we need to polish this, also create a calibration sceme
  Serial.println("touch ok?");

  //we do that later
  // if (!SD.begin(SD_CS)) {

  // }

  //init neopixels
  pixels.begin();
  pixels.clear();
  pixels.setBrightness(5);
  pixels.show();
  // pixels.fill(pixels.Color(255,0,0), 0, 2);
  // pixels.fill(pixels.Color(255,255,0), 3, 14);
  Serial.println("neopixel ok");

  //serial for gps
  GPSSERIAL.begin(9600, SERIAL_8N1, 16, 17);
  while(!gps.available( GPSSERIAL ) && millis() < 5000){}
  if(gps.statistics.ok){
    Serial.println("gps ok");
  }else{
    Serial.println("gps fail");
  }
  

  //serial for bluetooth
  //Serial.begin(115200);

  //i2c for sensors
  // Wire.setSCL(PB10);
  // Wire.setSDA(PB11);
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS);
  mpu_ok = Wire.endTransmission() == 0;

  //initialize mpu
  //mpu_ok = mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  if(mpu_ok){
    mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
    Serial.println("MPU ok");
  }else{
    Serial.println("MPU failled");
  }
    

  Wire.beginTransmission(HMC5883L_ADDRESS);
  hmc_ok = Wire.endTransmission() == 0;
  // initialize compass
  //hmc_ok = compass.begin();
  if(hmc_ok){
    compass.begin();
    // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);
    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);
    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);
    // Set calibration offset. See HMC5883L_calibration.ino
    compass.setOffset(0, 0); 
    Serial.println("HMC ok");
  }else{
    Serial.println("HMC failled");
  }

  //initialize temp/humid sensor
  tmp_ok = aht10.begin();
  if(tmp_ok){
    Serial.println("TMP ok");
  }else{
    Serial.println("TMP failled");
  }

  pinMode(SD_DET, INPUT_PULLUP);
  if(digitalRead(SD_DET) == HIGH){
    Serial.println("sdcard not present");
  }else{
    Serial.println("sdcard pressent");
  }

  random_startup = random(0, 10);

  //playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false);
  //playAnim(NULL, neoAnimFPS, 0, false);

  Serial.println("END SETUP");

}

void loop() {

  lighting();
  handle_sd();
  handle_gps();
  handle_can();
  if(hmc_ok && mpu_ok && tmp_ok){
    handle_sensors();
  }
  


}

void handle_gps(){

  // if(GPSSERIAL.available()){
  //   valid_gps = gps.encode(GPSSERIAL.read());
  // }
  if(gps.available( GPSSERIAL )) {
    //fix = gps.read();
    if(gps.fix().status != fix.status){
      switch (gps.fix().status){
        case 0:
          Serial.println("Lost/No Fix!");
        break;
        default:
          Serial.print("Got Fix! ");
          Serial.print(gps.fix().satellites);
          Serial.println(" Sats");
        break;

      }
    }
    fix = gps.read();
  }

}


void handle_sd(){

  if(!digitalRead(SD_DET) && !sd_ok && !sd_err){
    delay(SD_DET_DELAY);
    if(!SD.begin(SD_CS)){
      sd_ok = 0;
      sd_err = 1;
      Serial.println("SDCARD ERROR!");
    }else{
      sd_ok = 1;
      Serial.println("SDCARD MOUNTED!");
    }
  }else if(digitalRead(SD_DET) && (sd_ok || sd_err)){
    SD.end();
    sd_ok = 0;
    sd_err = 0;
    Serial.println("SDCARD EJECTED");
  }
  
  if(sd_ok && !start_anim_finished){

    for(int i; i<10;i++){
      if(SD.exists(anim_filenames[i])){
        animNumber++;
      }
    }
    
    //Serial.println(animNumber);
    //randomSeed(analogRead(36));
    uint8_t anim_number = random(0, animNumber);
    //anim_number -= 1;
    //Serial.println(anim_number);
    if(SD.exists(anim_filenames[anim_number])){
      myFile = SD.open(anim_filenames[anim_number]);
      // Serial.print("oppened file ");
      // Serial.println(anim_filenames[anim_number]);
      sdAnimFPS = myFile.parseInt();
      sdAnimBrightness = myFile.parseInt();
      sdAnimLength = myFile.parseInt();
      for(int i; i < sdAnimLength * NUMPIXELS; i++){
        sdpixeldata[i] = myFile.parseInt();
      }
      myFile.close();
      pixels.setBrightness(sdAnimBrightness);
      playAnim( sdpixeldata , sdAnimFPS, sdAnimLength, false);
      start_anim_finished = 1;
    }else{
      // Serial.println("no animation exist");
      playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false);
      start_anim_finished = 1;
    }
  }else if(sd_err && !start_anim_finished){
    playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false);
    start_anim_finished = 1;
  }

}

// Begin playing a NeoPixel animation from a PROGMEM table
void playAnim(const uint32_t *addr, uint8_t fps, uint16_t bytes, bool repeat) {
  pixelBaseAddr = addr;// ignore the error with -fpermissive
  if(addr) {
    pixelFPS    = fps;
    pixelLen    = bytes * NUMPIXELS;
    pixelLoop   = repeat; //if set to 'repeat' it'll loop, set to 0 to play once only
    pixelIdx    = 0;
  } else {
    //pixels.clear();
  }
}

uint32_t prev = 0;
void lighting(){
  if(!pixels.canShow()){
    return;
  }
    
  uint32_t t;      // Current time in milliseconds
  // Until the next animation frame interval has elapsed...
  if( ((t = millis()) - prev) > (1000 / (pixelFPS+1)) ){
    pixels.show();
    prev = t; // Save refresh time for next frame sync

    if(pixelBaseAddr) {
      for(uint8_t i=0; i<NUMPIXELS; i++) { // For each NeoPixel...
        // Read pixel color from PROGMEM table
        uint32_t rgb = pgm_read_dword(&pixelBaseAddr[pixelIdx++]);
        // Expand 16-bit color to 24 bits using gamma tables
        // RRRRRGGGGGGBBBBB -> RRRRRRRR GGGGGGGG BBBBBBBB
        pixels.setPixelColor(i,
          ( rgb >> 16  & 0xFF),
          ( rgb >>  8 & 0xFF ),
          ( rgb        & 0xFF));
          
      }
      
        if(pixelIdx >= pixelLen) { // End of animation table reached
          if(pixelLoop) { // Repeat animation
            pixelIdx = 0; // Reset index to start of table
          } else {        // else switch off LEDs
            playAnim(NULL, neoAnimFPS, 0, false);
          }
        } 
    
    }
  }

}


void handle_can(){

  if(millis() - can_tm > can_refresh){
    can_tm = millis();

    if(!canWaitingMsg){
      switch(msgType){
        case 0://engine rpm
          tx_frame.data.u64 = 0x2 | (0x01 << 8) | (0x0c << 16) | 0xAAAAAAAAAA000000;
        break;
        case 1://coolant temp
          tx_frame.data.u64 = 0x2 | (0x01 << 8) | (0x05 << 16) | 0xAAAAAAAAAA000000;
        break;
        case 2:
          tx_frame.data.u64 = 0x2 | (0x01 << 8) | (0x42 << 16) | 0xAAAAAAAAAA000000;
        break;
      }

      // errorType = CAN.sendMessage(&outMSG);
      // if(errorType){
      //   Serial.println(errorType);
      //   errorType = 0;
      // }else{
      //   canWaitingMsg = 1;
      // }
      if(ESP32Can.CANWriteFrame(&tx_frame)){
        Serial.println("canTXerror");
      }else{
        canWaitingMsg = 1;
      }
      

    }else if(canWaitingMsg && xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 10) == pdTRUE){
      // errorType = CAN.readMessage(&inMSG);
      // if(errorType){
      //   Serial.println(errorType);
      //   errorType = 0;
      // }
      if( rx_frame.MsgID == 0x7e8 && rx_frame.data.u8[1] & 0x40){
        switch(msgType){
          case 0:
            // engineRPM = ((inMSG.data[3] << 8) | inMSG.data[4]) * 0.25;
            engineRPM = ((rx_frame.data.u64 & 0xFFFF000000) >> 24) *0.25;
          break;
          case 1:
            // engineTemp = (inMSG.data[3]) - 40;
            engineTemp = ((rx_frame.data.u64 & 0xFF000000) >> 24) - 40;
          break;
          case 2:
            // ecuVoltage = ((inMSG.data[3] << 8) | inMSG.data[4]) * 0.001;
            ecuVoltage = ((rx_frame.data.u64 & 0xFFFF000000) >> 24) * 0.001;
          break;
        }
        canWaitingMsg = 0;
        msgType++;
        if(msgType > 2){
          msgType = 0;
        }
      }
    }

  }

}


// No tilt compensation
float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}
 
// Tilt compensation
float tiltCompensate(Vector mag, Vector normAccel)
{
  // Pitch & Roll 
  
  float roll;
  float pitch;
  
  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }
  
    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  
  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
 
  float heading = atan2(Yh, Xh);
    
  return heading;
}

// Correct angle
float correctAngle(float heading)
{
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
}

#define sensorsPoolrate 100
uint32_t tmSensors;
float heading1;
float heading2;

void handle_sensors(){

  if(millis() - tmSensors > sensorsPoolrate){
    tmSensors = millis();

    Vector mag = compass.readNormalize();
    Vector acc = mpu.readScaledAccel();  

    // Calculate headings
    heading1 = noTiltCompensate(mag);
    heading2 = tiltCompensate(mag, acc);

    if (heading2 == -1000)
  {
    heading2 = heading1;
  }

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;
  
  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180/M_PI; 
  heading2 = heading2 * 180/M_PI; 

  }
}

