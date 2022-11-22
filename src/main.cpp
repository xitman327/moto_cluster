#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <STM32RTC.h>
STM32RTC& rtc = STM32RTC::getInstance();

#include <Adafruit_GFX.h>
// #include <ILI9488.h>
#define TFT_CS         PB12
#define TFT_DC         PA15
#define TFT_LED        PA8
#define TFT_RST        -1
// ILI9488 tft = ILI9488(TFT_CS, TFT_DC, TFT_RST);
// #include <Adafruit_PCD8544.h>
// Adafruit_PCD8544 display = Adafruit_PCD8544(TFT_DC, TFT_CS, TFT_RST);

#include <XPT2046.h>
XPT2046 touch(/*cs=*/ PB3, /*irq=*/ PB4);

#include <SD.h>
#define SD_CS PB7
#define SD_DET PB8
File myFile;
#define SD_DET_DELAY 10
bool sd_ok, sd_err;

#include <mcp2515.h>
MCP2515 CAN(PB6, 10000000UL, &SPI);
// #include <mcp_can.h>
// MCP_CAN CAN0(PB6);  //CAN CS
#define CAN0_INT PB5

#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN PB9
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
HardwareSerial GPSSERIAL(PA10, PA9);
#include <NMEAGPS.h>
//#include <GPSport.h>
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
bool valid_gps;


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

uint8_t random_startup;

bool start_anim_finished;
byte anim_i;
uint32_t default_anim[255] = {};

void lighting();
void handle_gps();
void handle_sd();
void playAnim(const uint32_t *addr, uint8_t fps, uint16_t bytes, bool repeat);

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
  SPI.setMISO(PB14);
  SPI.setMOSI(PB15);
  SPI.setSCLK(PB13);
  SPI.setSSEL(PB12);
  // SPI.setClockDivider(SPI_CLOCK_DIV8);
  //SPI.begin();

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
  // y = CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  // if(y == CAN_OK){
  //   BTSERAIL.println("can ok");
  // }else{
  //   BTSERAIL.print("can failled ");
  //   BTSERAIL.println(y);
  // }

  //init LCD
  // tft.begin();
  // uint8_t x = tft.readcommand8(ILI9488_RDMODE);
  // if(x == 0xff || x == 0x00){
  //   BTSERAIL.println("tft failled");
  //   tft_ok = 0;
  // }else{
  //   BTSERAIL.println("tft ok");
  //   tft_ok = 1;
  //   tft.setRotation(0);
  // }
  // display.begin();
  // display.clearDisplay();
  // display.print("hello world!");
  // BTSERAIL.println("lcd ok");
  

  //init touch
  touch.begin(480, 320);
  touch.setRotation(touch.ROT0);
  touch.setCalibration(209, 1759, 1775, 273);//we need to polish this, also create a calibration sceme
  BTSERAIL.println("touch ok?");

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
  BTSERAIL.println("neopixel ok");

  //serial for gps
  GPSSERIAL.begin(9600);
  while(!gps.available( GPSSERIAL )){}
  if(gps.statistics.ok){
    BTSERAIL.println("gps ok");
  }else{
    BTSERAIL.println("gps fail");
  }
  

  //serial for bluetooth
  //BTSERAIL.begin(115200);

  //i2c for sensors
  Wire.setSCL(PB10);
  Wire.setSDA(PB11);
  Wire.begin();

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

  pinMode(SD_DET, INPUT_PULLUP);
  if(digitalRead(SD_DET) == HIGH){
    BTSERAIL.println("sdcard not present");
  }else{
    BTSERAIL.println("sdcard pressent");
  }

  random_startup = random(0, 10);

  //playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false);

  BTSERAIL.println("END SETUP");

}

void loop() {

  lighting();
  handle_sd();
  handle_gps();

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
          BTSERAIL.println("Lost/No Fix!");
        break;
        default:
          BTSERAIL.print("Got Fix! ");
          BTSERAIL.print(gps.fix().satellites);
          BTSERAIL.println(" Sats");
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
      BTSERAIL.println("SDCARD ERROR!");
    }else{
      sd_ok = 1;
      BTSERAIL.println("SDCARD MOUNTED!");
    }
  }else if(digitalRead(SD_DET) && (sd_ok || sd_err)){
    SD.end();
    sd_ok = 0;
    sd_err = 0;
    BTSERAIL.println("SDCARD EJECTED");
  }

  if(sd_ok && !start_anim_finished){
    
    if(SD.exists("/ANI/0.txt")){
      myFile = SD.open("/ANI/0.txt");
      BTSERAIL.println("oppened file /ANI/0.txt");
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
      BTSERAIL.println("/ANI/0.txt does not exist");
      playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false);
      start_anim_finished = 1;
    }
  }else if(sd_err && !start_anim_finished){
    BTSERAIL.println("no sd animation");
    playAnim( paternPixelData , neoAnimFPS, ANIM_LENGTH, false);
    start_anim_finished = 1;
  }

}

// Begin playing a NeoPixel animation from a PROGMEM table
void playAnim(const uint32_t *addr, uint8_t fps, uint16_t bytes, bool repeat) {
  pixelBaseAddr = addr;
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
  //while(((t = millis()) - prev) < (1000 / pixelFPS));

  if( ((t = millis()) - prev) > (1000 / pixelFPS) ){
      // Show LEDs rendered on prior pass.  It's done this way so animation timing
      // is a bit more consistent (frame rendering time may vary slightly).
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