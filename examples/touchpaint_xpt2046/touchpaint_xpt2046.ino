/***************************************************
  This version of touchscreen painting is setup to work with PJRC touch
  screen(http://pjrc.com/store/display_ili9341_touch.html), which uses a 
  different touch controller thatn the Adafruit LI9341 Shield: 
  
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


//****************************************************************************
// Touch options, Defaults to SPI
//****************************************************************************
// uncomment to try Touch on SPI1 pins
//#define TOUCH_SPI1
// FlexSPI on SPI1 pins
//#define TOUCH_FLEXSPI


//****************************************************************************
// Includes
//****************************************************************************
#include <XPT2046_Touchscreen.h>
#include <ILI9341_GIGA_n.h>

//****************************************************************************
// This is calibration data for the raw touch data to the screen coordinates
//****************************************************************************
// Warning, These are
#define TS_MINX 337
#define TS_MINY 529
#define TS_MAXX 3729
#define TS_MAXY 3711

//****************************************************************************
// Settings and objects
//****************************************************************************
#define TFT_DC 9
#define TFT_RST 8
#define TFT_CS 10
#define TOUCH_CS  6

XPT2046_Touchscreen ts(TOUCH_CS);
ILI9341_GIGA_n tft(&SPI1, TFT_CS, TFT_DC, TFT_RST);

// Size of the color selection boxes and the paintbrush size
#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;

//****************************************************************************
// Setup
//****************************************************************************
void setup(void) {
  while (!Serial && (millis() <= 1000));

  Serial.begin(115200);
  Serial.println(F("Touch Paint!"));

  tft.begin();

  if (!ts.begin(SPI1)) {
    Serial.println("Couldn't start touchscreen controller");
    while (1);
  }
  Serial.println("Touchscreen started");

  tft.fillScreen(ILI9341_BLACK);

  // make the color selection boxes
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
  tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, ILI9341_GREEN);
  tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, ILI9341_CYAN);
  tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, ILI9341_BLUE);
  tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);

  // select the current color 'red'
  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
  currentcolor = ILI9341_RED;
}


void loop()
{
  // See if there's any  touch data for us
  if (ts.bufferEmpty()) {
    return;
  }

  // You can also wait for a touch
  /*
  if (! ts.touched()) {
    return;
  }
*/

  // Retrieve a point
  TS_Point p = ts.getPoint();

  // p is in ILI9341_t3 setOrientation 1 settings. so we need to map x and y differently.
/*
  Serial.print("X = "); Serial.print(p.x);
  Serial.print("\tY = "); Serial.print(p.y);
  Serial.print("\tPressure = "); Serial.print(p.z);
*/

  // Scale from ~0->4000 to tft.width using the calibration #'s
#ifdef SCREEN_ORIENTATION_1
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
#else
  
  uint16_t px = map(p.y, TS_MAXY, TS_MINY, 0, tft.width());
  p.y = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
  p.x = px;
#endif
/*  
    Serial.print(" ("); Serial.print(p.x);
    Serial.print(", "); Serial.print(p.y);
    Serial.println(")");
*/  

  if (p.y < BOXSIZE) {
    oldcolor = currentcolor;

    if (p.x < BOXSIZE) {
      currentcolor = ILI9341_RED;
      tft.drawRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
    } else if (p.x < BOXSIZE * 2) {
      currentcolor = ILI9341_YELLOW;
      tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
    } else if (p.x < BOXSIZE * 3) {
      currentcolor = ILI9341_GREEN;
      tft.drawRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
    } else if (p.x < BOXSIZE * 4) {
      currentcolor = ILI9341_CYAN;
      tft.drawRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
    } else if (p.x < BOXSIZE * 5) {
      currentcolor = ILI9341_BLUE;
      tft.drawRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
    } else if (p.x < BOXSIZE * 6) {
      currentcolor = ILI9341_MAGENTA;
      tft.drawRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
    }

    if (oldcolor != currentcolor) {
      if (oldcolor == ILI9341_RED)
        tft.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
      if (oldcolor == ILI9341_YELLOW)
        tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
      if (oldcolor == ILI9341_GREEN)
        tft.fillRect(BOXSIZE * 2, 0, BOXSIZE, BOXSIZE, ILI9341_GREEN);
      if (oldcolor == ILI9341_CYAN)
        tft.fillRect(BOXSIZE * 3, 0, BOXSIZE, BOXSIZE, ILI9341_CYAN);
      if (oldcolor == ILI9341_BLUE)
        tft.fillRect(BOXSIZE * 4, 0, BOXSIZE, BOXSIZE, ILI9341_BLUE);
      if (oldcolor == ILI9341_MAGENTA)
        tft.fillRect(BOXSIZE * 5, 0, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);
    }
  }
  if (((p.y - PENRADIUS) > BOXSIZE) && ((p.y + PENRADIUS) < tft.height())) {
    tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
  }
}
