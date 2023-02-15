#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <FS.h>
#include <SD.h>
#define SD_CS 15
#define SD_DET 13
File myFile;
#define SD_DET_DELAY 10
bool sd_ok, sd_err;
//const char* anim_filenames[10] PROGMEM={"/ANI/0.txt", "/ANI/1.txt", "/ANI/2.txt", "/ANI/3.txt", "/ANI/4.txt","/ANI/5.txt","/ANI/6.txt","/ANI/7.txt","/ANI/8.txt","/ANI/9.txt"};
uint8_t animNumber;


// #include <STM32RTC.h>
// STM32RTC& rtc = STM32RTC::getInstance();

#include <TFT_eSPI.h>      // Graphics library
TFT_eSPI tft = TFT_eSPI(); // Invoke library

// #include <Adafruit_GFX.h>
// #include <ILI9488.h>
// #define TFT_CS         26
// #define TFT_DC         27
 #define TFT_LED        32
// #define TFT_RST        -1
// ILI9488 tft = ILI9488(TFT_CS, TFT_DC, TFT_RST);
// #include <Adafruit_PCD8544.h>
// Adafruit_PCD8544 display = Adafruit_PCD8544(TFT_DC, TFT_CS, TFT_RST);

#include <XPT2046.h>
XPT2046 touch(/*cs=*/ 25, /*irq=*/ 14);

#include "gauge.h"

#include <CAN.h> // the OBD2 library depends on the CAN library
#include <OBD2.h>
#define CAN_CS 5
#define CAN_INT 4
uint8_t msgType, errorType;
bool canWaitingMsg;
#define can_refresh 200
uint32_t can_tm;

int16_t engineRPM = 69, engineTemp;
double ecuVoltage;


#define NEOPIXEL_PIN 12
#define NUMPIXELS 15 
#include <Adafruit_NeoPixel.h>
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
                  audioLoop,     // If true, audio repeats
                  inpgm;


// #include <TinyGPS.h>
// TinyGPS gps;
#define GPSSERIAL Serial2
// #include <NMEAGPS.h>
//#include <GPSport.h>
// NMEAGPS  gps; // This parses the GPS characters
// gps_fix  fix; // This holds on to the latest values
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
bool valid_gps;


#include <MPU6050.h>
MPU6050 mpu;  


#include <HMC5883L.h>
HMC5883L compass;


#include <AHTxx.h>
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR);



#define LIGHT_SENSOR 33
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
void playAnim(const uint32_t *addr, uint8_t fps, uint16_t bytes, bool repeat, bool isPGM = true);
void handle_can();
void handle_sensors();
void displayInfo();
void handle_bright();
void handle_ui();
 
extern int ringMeter();

void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg)
{
  int16_t x1, y1;
  uint16_t wid, ht;
  //tft.setFont(f);
  tft.setCursor(x, y);
  tft.setTextSize(sz);
  tft.println(msg);
}

int a = 1000;
int b = 4000;
int j, j2;
int i, i2;

TaskHandle_t Task1;
void Task1code( void * parameter) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  //tft.setTextSize(3);

  //tft.drawRGBBitmap(0, 0, test, 480, 320);
  // tft.drawBitmap(0,0, &test,480,320,TFT_BLACK);



   CAN.setPins(CAN_CS, CAN_INT);
  if (!OBD2.begin()) {
    Serial.println(F("CAN failed!"));
  } else {
    Serial.println(F("CAN success"));
    OBD2.setTimeout(500);
  }

  // move them to a looping function
  // sprintf(m_pElemXRingGauge1->pElem->pStrBuf, "%d", engineRPM);
  // gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, engineRPM);

  for(;;) { //loop
    handle_sd();
    handle_can();
    handle_ui();
  }
}

void setup() {

  //init first for debugging
  Serial.begin(115200);
  Serial.println("starting...........");
  Serial.println("Serial ok");

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */
      
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  pinMode(LIGHT_SENSOR, ANALOG);
  pinMode(TFT_LED, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(TFT_LED, 0);
   

  //init neopixels
  pixels.begin();
  pixels.clear();
  pixels.setBrightness(5);
  //pixels.setPixelColor(2, 0xffff00);
  pixels.show();
  // pixels.fill(pixels.Color(255,0,0), 0, 2);
  // pixels.fill(pixels.Color(255,255,0), 3, 14);
  Serial.println("neopixel ok");

  //serial for gps
  GPSSERIAL.begin(9600, SERIAL_8N1, 16, 17);
  while(!GPSSERIAL.available() && millis() < 5000){}
  if(gps.encode(GPSSERIAL.read())){
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
  //Serial.println(mpu_ok);
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

  Wire.beginTransmission(AHTXX_ADDRESS_X38);
  tmp_ok = Wire.endTransmission() == 0;
  //initialize temp/humid sensor
  if(tmp_ok){
    aht10.begin();
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

  Serial.println("END SETUP");

}

void loop() {

  lighting();
  handle_gps();
  if(hmc_ok && mpu_ok){
    handle_sensors();
  }
  handle_bright();


}

#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

#define ui_refresh 100
uint32_t ui_tm;
bool ui_start;
int reading = 0; // Value to be displayed
int d = 0; // Variable used for the sinewave test waveform
void handle_ui(){
  if(millis() - ui_tm > ui_refresh){
      ui_tm = millis();

    if(!ui_start){
      ui_start = 1;
    }

    // Test with a slowly changing value from a Sine function
    d += 5; if (d >= 360) d = 0;

    // Set the the position, gap between meters, and inner radius of the meters
    int xpos = 0, ypos = 5, gap = 4, radius = 52;

    // Draw meter and get back x position of next meter

    // Test with Sine wave function, normally reading will be from a sensor
    reading = 250 + 250 * sineWave(d+0);
    //xpos = gap + ringMeter(reading, 0, 500, xpos, ypos, radius, "mA", GREEN2RED); // Draw analogue meter

    //reading = 20 + 30 * sineWave(d+60);
    //xpos = gap + ringMeter(reading, -10, 50, xpos, ypos, radius, "degC", BLUE2RED); // Draw analogue meter

    //reading = 50 + 50 * sineWave(d + 120);
    //ringMeter(reading, 0, 100, xpos, ypos, radius, "%RH", BLUE2BLUE); // Draw analogue meter


    // Draw two more larger meters
    //xpos = 20, ypos = 115, gap = 24, radius = 64;

    //reading = 1000 + 150 * sineWave(d + 90);
    //xpos = gap + ringMeter(reading, 850, 1150, xpos, ypos, radius, "mb", BLUE2RED); // Draw analogue meter

    //reading = 15 + 15 * sineWave(d + 150);
    //xpos = gap + ringMeter(reading, 0, 30, xpos, ypos, radius, "Volts", GREEN2GREEN); // Draw analogue meter

    // Draw a large meter
    xpos = 40, ypos = 5, gap = 15, radius = 120;
    //reading = 175;
    // Comment out above meters, then uncomment the next line to show large meter
    ringMeter(reading,0,500, xpos,ypos,radius," Watts",GREEN2RED); // Draw analogue meter

  }
}


#define brt_refresh 100
uint32_t brt_tm;
int bright_min = 10, bright_max = 255, brightness;
void handle_bright(){
  if(millis() - brt_tm > brt_refresh){
      brt_tm = millis();
      brightness = map(analogRead(LIGHT_SENSOR), 0, 4095, bright_max, bright_min);
      ledcWrite(0, brightness);
      pixels.setBrightness(brightness);
    }
}


#define tm_gps 500
uint32_t tmgps;
void handle_gps(){

  if(millis() - tmgps > tm_gps){
    tmgps = millis();
    displayInfo();
  }
  if( GPSSERIAL.available() ) {
    gps.encode(GPSSERIAL.read());
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
    char tms[59];
    for(int i; i<10;i++){
      sprintf(tms, "/ANI/%d.txt", i);
      if(SD.exists(tms)){
        animNumber++;
      }
    }
    //animNumber = 2;
    Serial.println(animNumber);
    //randomSeed(analogRead(36));
    uint8_t anim_number = random(0, animNumber);
    //anim_number -= 1;
    //Serial.println(anim_number);
    sprintf(tms, "/ANI/%d.txt", anim_number);
    if(SD.exists( tms)){
      myFile = SD.open(tms);
      Serial.print("oppened file ");
      Serial.println(tms);
      sdAnimFPS = myFile.parseInt();
      // Serial.println(sdAnimFPS);
      sdAnimBrightness = myFile.parseInt();
      // Serial.println(sdAnimBrightness);
      sdAnimLength = myFile.parseInt();
      // Serial.println(sdAnimLength);
      for(int i = 0; i < sdAnimLength * NUMPIXELS; i++){
        sdpixeldata[i] = myFile.parseInt();
        // Serial.println(sdpixeldata[i]);
      }
      myFile.close();
      pixels.setBrightness(sdAnimBrightness);
      playAnim( sdpixeldata , sdAnimFPS, sdAnimLength, false, false);
      start_anim_finished = 1;
    }else{
      Serial.println("no animation exist");
      playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false, true);
      start_anim_finished = 1;
    }
  }else if((sd_err || digitalRead(SD_DET)) && !start_anim_finished){
    Serial.println("no sd exist");
    playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false, true);
    start_anim_finished = 1;
  }

}

// Begin playing a NeoPixel animation from a PROGMEM table
void playAnim(const uint32_t *addr, uint8_t fps, uint16_t bytes, bool repeat, bool isPGM) {
  pixelBaseAddr = (uint32_t*)addr;// ignore the error with -fpermissive
  if(addr) {
    pixelFPS    = fps;
    pixelLen    = bytes * NUMPIXELS;
    pixelLoop   = repeat; //if set to 'repeat' it'll loop, set to 0 to play once only
    pixelIdx    = 0;
    inpgm = isPGM;
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
        uint32_t rgb;
        if(inpgm){
          rgb = pgm_read_dword(&pixelBaseAddr[pixelIdx++]);
        }else{
          rgb = pixelBaseAddr[pixelIdx++];
        }
        //Serial.println(rgb);
        pixels.setPixelColor(i,
          ( rgb >> 16  & 0xFF),
          ( rgb >>  8 & 0xFF ),
          ( rgb        & 0xFF));
          
      }
      
      if(pixelIdx >= pixelLen) { // End of animation table reached
        if(pixelLoop) { // Repeat animation
          pixelIdx = 0; // Reset index to start of table
          Serial.println("animation repeat");
        } else {        // else switch off LEDs
          playAnim(NULL, neoAnimFPS, 0, false, false);
          Serial.println("animation end");
        }
      } 
    }
  }

}

void handle_can(){

  if(millis() - can_tm > can_refresh){
    can_tm = millis();
    switch(msgType){
      case 0:
        engineRPM = (int16_t)OBD2.pidRead(ENGINE_RPM);
        break;

      case 1:
        engineTemp = (int16_t)OBD2.pidRead(ENGINE_COOLANT_TEMPERATURE);
      break;

      case 2:
        ecuVoltage = (double)OBD2.pidRead(CONTROL_MODULE_VOLTAGE);
      break;

      default:
        break;
    }
    msgType++;
    if(msgType > 2){
      msgType = 0;
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
  float declinationAngle = (5.0 + (14.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;
  
  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180/M_PI; 
  heading2 = heading2 * 180/M_PI; 

  // Serial.print(heading1);
  // Serial.print(" : ");
  // Serial.println(heading2);

  }
}

void displayInfo()
{

  // Serial.printf("Location: %d,%d %s  Date/Time: D%d M%d Y%d %s H%d M%d S%d %s\r", \
  //                                                   gps.location.lat(),\
  //                                                   gps.location.lng(),\
  //                                                   gps.location.isValid()?"Valid":"invalid",\
  //                                                   gps.date.day(),\
  //                                                   gps.date.month(),\
  //                                                   gps.date.year(),\
  //                                                   gps.date.isValid()?"Valid":"invalid",\
  //                                                   gps.time.hour(),\
  //                                                   gps.time.minute(),\
  //                                                   gps.time.second(),\
  //                                                   gps.time.isValid()?"Valid":"invalid"\
  //                                                     );
  // Serial.print(F("Location: ")); 
  // if (gps.location.isValid())
  // {
  //   Serial.print(gps.location.lat(), 6);
  //   Serial.print(F(","));
  //   Serial.print(gps.location.lng(), 6);
  // }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }

  // Serial.print(F("  Date/Time: "));
  // if (gps.date.isValid())
  // {
  //   Serial.print(gps.date.month());
  //   Serial.print(F("/"));
  //   Serial.print(gps.date.day());
  //   Serial.print(F("/"));
  //   Serial.print(gps.date.year());
  // }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }

  // Serial.print(F(" "));
  // if (gps.time.isValid())
  // {
  //   if (gps.time.hour() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.hour());
  //   Serial.print(F(":"));
  //   if (gps.time.minute() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.minute());
  //   Serial.print(F(":"));
  //   if (gps.time.second() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.second());
  //   Serial.print(F("."));
  //   if (gps.time.centisecond() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.centisecond());
  // }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }

  //Serial.println();
}

void parse_ui(){
  Serial.println("Parsing ui...");

  if(sd_ok){
    if(SD.exists("/ui1.txt")){
      File uiFile = SD.open("/ui1.txt");
      //uiFile.
    }
  }
}
