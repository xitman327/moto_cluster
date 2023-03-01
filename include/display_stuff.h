#include <Arduino.h>
#include <TFT_eSPI.h>
#include <FS.h>
#include "LittleFS.h"
#define SPIFFS LittleFS

TFT_eSPI tft = TFT_eSPI();

#define CALIBRATION_FILE "/calibrationData"

uint16_t calibrationData[5];
uint8_t calDataOK = 0;

uint32_t fps, fps_tm;

#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

#define ui_refresh 10
uint32_t ui_tm;
bool ui_start;
int reading = 0; // Value to be displayed
int d = 0; // Variable used for the sinewave test waveform
int xpos = 0, ypos = 5, gap = 4, radius = 52;
int8_t ramp = 1;
uint16_t touch_x = 0, touch_y = 0;

float sineWave(int phase);
unsigned int rainbow(byte value);
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme);

void init_lcd(void){

    tft.begin();
    tft.setRotation(1);

    //check if calibration file exists
    if (SPIFFS.exists(CALIBRATION_FILE)) {
        log_esp("file exits");
        File f = SPIFFS.open(CALIBRATION_FILE, "r");
        if (f) {
        if (f.readBytes((char *)calibrationData, 14) == 14){
            calDataOK = 1;
            log_esp("cal data ok");
        }
        f.close();
        }
    }
    if (calDataOK) {
        // calibration data valid
        tft.setTouch(calibrationData);
        log_esp("cal ok");
    } else {
        log_esp("begin cal");
        // data not valid. recalibrate
        tft.calibrateTouch(calibrationData, TFT_WHITE, TFT_RED, 15);
        // store data
        File f = SPIFFS.open(CALIBRATION_FILE, "w");
        if (f) {
        f.write((const unsigned char *)calibrationData, 14);
        f.close();
        log_esp("begin cal ok");
        }
    }

    tft.fillScreen(TFT_BLACK);

}

void handle_ui(){
  if(millis() - ui_tm > ui_refresh){
      ui_tm = millis();
    fps_tm = millis();

    if(!ui_start){
      ui_start = 1;
      xpos = 480/2 - 160, ypos = 0, gap = 15, radius = 170;

    }

    // if (tft.getTouch(&touch_x, &touch_y)) {
    //   Serial.printf("TouchpointX:%d Y:%d \n", touch_x, touch_y);
    //   tft.drawPixel(touch_x, touch_y, TFT_GREEN);
    //   tft.drawPixel(touch_x, touch_y+1, TFT_GREEN);
    //   tft.drawPixel(touch_x+1, touch_y, TFT_GREEN);
    //   tft.drawPixel(touch_x+1, touch_y+1, TFT_GREEN);
    // }


    // Draw a large meter
    reading +=(ramp);
    if (reading>98) ramp = -1;
    if (reading<0) ramp = 1;
    // Comment out above meters, then uncomment the next line to show large meter
    ringMeter(reading,0,100, xpos,ypos,radius," Km/h",BLUE2RED); // Draw analogue meter

    fps = (millis() - fps_tm);
    tft.setCursor(0,0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.print(1 / ((double)fps / 1000.0));
    tft.print(" fps  ");

  }
}

// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme)
{
  //draw the full meter every time. honestly the is wastefull, better do a sweap for visuals
  bool _fulldraw = false;


  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  
  x += r; y += r;   // Calculate coords of centre of ring

  int w = r / 3;    // Width of outer ring is 1/4 of radius
  
  int angle = 150;  // Half the sweep angle of meter (300 degrees)

  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

  byte seg = 5; // Segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 5; // Draw segments every 3 degrees, increase to 6 for segmented ring

  // Variable to save "value" text colour from scheme and set default
  int colour = TFT_BLUE;
 
  // Draw colour blocks every inc degrees
  for (int i = -angle+inc/2; i < angle-inc/2; i += inc) {

    if(_fulldraw || (i > v - inc - 1 && i < v + inc + 1) ){
      // Calculate pair of coordinates for segment start
      float sx = cos((i - 90) * 0.0174532925);
      float sy = sin((i - 90) * 0.0174532925);
      uint16_t x0 = sx * (r - w) + x;
      uint16_t y0 = sy * (r - w) + y;
      uint16_t x1 = sx * r + x;
      uint16_t y1 = sy * r + y;

      // Calculate pair of coordinates for segment end
      float sx2 = cos((i + seg - 90) * 0.0174532925);
      float sy2 = sin((i + seg - 90) * 0.0174532925);
      int x2 = sx2 * (r - w) + x;
      int y2 = sy2 * (r - w) + y;
      int x3 = sx2 * r + x;
      int y3 = sy2 * r + y;

      if (i < v) { // Fill in coloured segments with 2 triangles
        switch (scheme) {
          case 0: colour = TFT_RED; break; // Fixed colour
          case 1: colour = TFT_GREEN; break; // Fixed colour
          case 2: colour = TFT_BLUE; break; // Fixed colour
          case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
          case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break; // Green to red (high temperature etc)
          case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
          default: colour = TFT_BLUE; break; // Fixed colour
        }
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
        //text_colour = colour; // Save the last colour drawn
      }
      else // Fill in blank segments
      {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREY);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREY);
      }
    }

  }
  // Convert value to a string
  char buf[10];
  byte len = 3; if (value > 999) len = 5;
  dtostrf(value, len, 0, buf);
  buf[len] = ' '; buf[len+1] = 0; // Add blanking space and terminator, helps to centre text too!
  // Set the text colour to default
  tft.setTextSize(1);

  //   if (value<vmin || value>vmax) {
  //     drawAlert(x,y+90,50,1);
  //   }
  //   else {
  //     drawAlert(x,y+90,50,0);
  //   }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Uncomment next line to set the text colour to the last segment value!
  tft.setTextColor(colour, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  // Print value, if the meter is large then use big font 8, othewise use 4
  if (r > 84) {
    tft.setTextPadding(55*3); // Allow for 3 digits each 55 pixels wide
    tft.drawString(buf, x, y, 8); // Value in middle
  }
  else {
    tft.setTextPadding(3 * 14); // Allow for 3 digits each 14 pixels wide
    tft.drawString(buf, x, y, 4); // Value in middle
  }
  tft.setTextSize(1);
  tft.setTextPadding(0);
  // Print units, if the meter is large then use big font 4, othewise use 2
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (r > 84) tft.drawString(units, x, y + 60, 4); // Units display
  else tft.drawString(units, x, y + 15, 2); // Units display

  // Calculate and return right hand side x coordinate
  return x + r;
}

// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}

// #########################################################################
// Return a value in range -1 to +1 for a given phase angle in degrees
// #########################################################################
float sineWave(int phase) {
  return sin(phase * 0.0174532925);
}