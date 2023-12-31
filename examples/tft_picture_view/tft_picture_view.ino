
//=============================================================================
// Simple image (BMP optional JPEG and PNG) display program, which if the
// sketch is built with one of the USB Types which include MTP support
//=============================================================================
#include <SPI.h>
//#include <SD.h>
#include <elapsedMillis.h>
#include <FATFileSystem.h>
#include <Arduino_USBHostMbed5.h>
#include <GIGA_digitalWriteFast.h>


/***************************************************
  Some of this code originated with the spitftbitmap.ino sketch
  that is part of the Adafruit_ILI9341 library. 
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// warning, this sketch uses libraries that are not installed as part of Teensyduino
// ILI9341_t3n - https://github.com/KurtE/ILI9341_t3n
// JPGDEC - https://github.com/bitbank2/JPEGDEC (also on arduino library manager)
// PNGdec - https://github.com/bitbank2/PNGdec (also on arduino library manager)

// optional JPEG support requires external library
// uncomment if you wish to use.
#include <JPEGDEC.h>

// optional PNG support requires external library
#include <PNGdec.h>


// support for ILI9341_t3n - that adds additional features
#include <ILI9341_GIGA_n.h>


//-----------------------------------------------------------------------------
// ILI9341 displays
//-----------------------------------------------------------------------------

// This is calibration data for the raw touch data to the screen coordinates
// Warning, These may need to be tweeked.
#define TS_MINX 337
#define TS_MINY 529
#define TS_MAXX 3729
#define TS_MAXY 3711
#define USE_SPI1
#ifdef USE_SPI1
#define TFT_DC 9
#define TFT_RST 8
#define TFT_CS 7
ILI9341_GIGA_n tft(&SPI1, TFT_CS, TFT_DC, TFT_RST);
#else
#define TFT_DC 24
#define TFT_RST 26
#define TFT_CS 22
ILI9341_GIGA_n tft(&SPI, TFT_CS, TFT_DC, TFT_RST, DMA1_Stream3);
#endif

#define SUPPORTS_XPT2046_TOUCH

//-----------------------------------------------------------------------------
// XPT Touch screen
//-----------------------------------------------------------------------------
#if defined(TOUCH_CS) && defined(SUPPORTS_XPT2046_TOUCH)
#include <XPT2046_Touchscreen.h>
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_TIRQ);
#endif

//-----------------------------------------------------------------------------
// Some common things.
//-----------------------------------------------------------------------------
#define TFT_USE_FRAME_BUFFER

#ifndef BLUE
#define BLUE 0x001F
#define BLACK 0x0000
#define WHITE 0xFFFF
#define GREEN 0x07E0
#define RED 0xf800
#endif


//-----------------------------------------------------------------------------
// Other globals
//-----------------------------------------------------------------------------
int g_tft_width = 0;
int g_tft_height = 0;

bool g_use_efb = true;

USBHostMSD msd;
mbed::FATFileSystem usb("usb");

#define MAX_FILENAME_LEN 256

DIR *root_dir = nullptr;  //
//File rootFile;
FILE *myfile; // used for JPG and PNG file

bool g_fast_mode = false;
bool g_picture_loaded = false;

elapsedMillis emDisplayed;
#define DISPLAY_IMAGES_TIME 2500

// Options file information
static const PROGMEM char options_file_name[] = "/usb/PictureViewOptions.ini";
int g_debug_output = 0;
int g_stepMode = 0;
int g_BMPScale = -1;
int g_JPGScale = 0;
int g_PNGScale = 1;
int g_center_image = 1;
int g_display_image_time = 2500;
int g_background_color = BLACK;
int g_max_scale_up = 4;


// scale boundaries {2, 4, 8, 16<maybe>}
enum { SCL_HALF = 0,
       SCL_QUARTER,
       SCL_EIGHTH,
       SCL_16TH };
int g_jpg_scale_x_above[4];
int g_jpg_scale_y_above[4];

// variables used in some of the display output functions
int g_image_width;
int g_image_height;
int g_image_offset_x = 0;
int g_image_offset_y = 0;
uint8_t g_image_scale = 1;
uint8_t g_image_scale_up = 0;
uint32_t g_WRCount = 0;  // debug count how many time writeRect called




//****************************************************************************
// Setup
//****************************************************************************
void setup(void) {
  // Enable the USBHost
  pinMode(PA_15, OUTPUT);
  digitalWrite(PA_15, HIGH);

  // Keep the SD card inactive while working the display.
  delay(20);

  while (!Serial && millis() < 3000)
    ;
  // give chance to debug some display startups...

  //-----------------------------------------------------------------------------
  // initialize display
  //-----------------------------------------------------------------------------

  pinMode(TFT_CS, OUTPUT);
  digitalWriteFast(TFT_CS, HIGH);

#ifdef TOUCH_CS
  pinMode(TOUCH_CS, OUTPUT);
  digitalWriteFast(TOUCH_CS, HIGH);
  Serial.print("\n\n*** start up touch cs:");
  Serial.print(TOUCH_CS, DEC);
  Serial.print(" irq:");
  Serial.print(TOUCH_TIRQ, DEC)
    : Serial.println(" ***");
  ts.begin();
#endif


  Serial.println("*** start up ILI9341 ***");
  tft.begin(10000000u);
  tft.setRotation(1);

  tft.fillScreen(RED);
  delay(500);
  tft.fillScreen(GREEN);
  delay(500);
  tft.fillScreen(BLUE);
  delay(500);

  g_tft_width = tft.width();
  g_tft_height = tft.height();

  FillScreen(BLUE);

  // See if we can initialize the File system
  Serial.print("Mounting USB device... ");

  int err;
  elapsedMillis em = 10000;  // fire the first time through
  tft.setTextSize(2);

  while (!msd.connect()) {
      //while (!port.connected()) {
      delay(1000);
  }


  while ((err = usb.mount(&msd)) != 0) {
    if (em > 10000) {
      FillScreen(RED);
      tft.print("Error mounting USB devic: ");
      tft.print(err, DEC);
      Serial.print("Error mounting USB device ");
      Serial.println(err);
      em = 0;
    }
    delay(1000);
  }
  Serial.println("Mounted.");

  root_dir = opendir("/usb/");
  if (!root_dir) {
    FillScreen(RED);
    tft.print("Failed to open Root directory");
  }


#ifdef ADD_SD_SUPPORT
  tft.println(F("Init SD card..."));

  if (!SD.begin(SD_CS)) {
    tft.setTextSize(2);
    FillScreen(RED);
    while (!SD.begin(SD_CS)) {
      //Serial.println(F("failed to access SD card!"));
      tft.print("failed to access SD card on cs:%u!\n", SD_CS);
      delay(2000);
    }
  }
#endif

  Serial.begin(115200);


  Serial.println("OK!");
  emDisplayed = g_display_image_time;

  //-----------------------------------------------------------------------------
  // Initialize options and then read optional config file
  //-----------------------------------------------------------------------------
  g_jpg_scale_x_above[0] = (g_tft_width * 3) / 2;
  g_jpg_scale_x_above[1] = g_tft_width * 3;
  g_jpg_scale_x_above[2] = g_tft_width * 6;
  g_jpg_scale_x_above[3] = g_tft_width * 12;

  g_jpg_scale_y_above[0] = (g_tft_height * 3) / 2;
  g_jpg_scale_y_above[1] = g_tft_height * 3;
  g_jpg_scale_y_above[2] = g_tft_height * 6;
  g_jpg_scale_y_above[3] = g_tft_height * 12;
  FILE *optionsFile = fopen(options_file_name, "r+");
  if (optionsFile) {
    ProcessOptionsFile(optionsFile);
    fclose(optionsFile);
  }
  ShowAllOptionValues();

  tft.useFrameBuffer(true);
}

//****************************************************************************
// loop
//****************************************************************************
void loop() {
  // don't process unless time elapsed or g_fast_mode
  // Timing may depend on which type of display we are using...
  // if it has logical frame buffer, maybe as soon as we display an image,
  // try to load the next one, and then wait until the image time to
  // tell display to update...
  if (!g_fast_mode && !g_stepMode && (!g_picture_loaded) && (emDisplayed < (uint32_t)g_display_image_time)) return;

  //---------------------------------------------------------------------------
  // Find the next file to read in.
  //---------------------------------------------------------------------------
  if (!g_picture_loaded) {
    bool did_rewind = false;
    const char *name = nullptr;
    uint8_t name_len;
    bool bmp_file = false;
    bool jpg_file = false;
    bool png_file = false;

    Serial.println("\nLoop looking for image file");

    struct dirent *dir_entry;
    for (;;) {
      dir_entry = readdir(root_dir);
      if (!dir_entry) {
        if (did_rewind) break;  // only go around once.
        rewinddir(root_dir);
        continue;  // try again
      }
      // maybe should check file name quick and dirty
      name = dir_entry->d_name;
      name_len = strlen(name);
      if (!name) continue;

      if ((strcmp(&name[name_len - 4], ".bmp") == 0) || (strcmp(&name[name_len - 4], ".BMP") == 0)) bmp_file = true;
      if ((strcmp(&name[name_len - 4], ".jpg") == 0) || (strcmp(&name[name_len - 4], ".JPG") == 0)) jpg_file = true;
      if (stricmp(&name[name_len - 4], ".bmp") == 0) bmp_file = true;
      if (stricmp(&name[name_len - 4], ".jpg") == 0) jpg_file = true;
      if (stricmp(&name[name_len - 4], ".png") == 0) png_file = true;
      //if (stricmp(name, options_file_name) == 0) ProcessOptionsFile(imageFile);
      if (bmp_file || jpg_file || png_file) break;
    }

    //---------------------------------------------------------------------------
    // Found a file so try to process it.
    //---------------------------------------------------------------------------
    if (dir_entry) {

      elapsedMillis emDraw = 0;
      char file_name[MAX_FILENAME_LEN];
      strcpy(file_name, "/usb/");
      strcat(file_name, name);
      g_WRCount = 0;
      if (bmp_file) {
        bmpDraw(file_name, true);

#ifdef __JPEGDEC__
      } else if (jpg_file) {
        processJPGFile(file_name, true);
#endif

#ifdef __PNGDEC__
      } else if (png_file) {
        processPNGFile(file_name, true);
#endif
      }
      Serial.print("!!File:");
      Serial.print(file_name);
      Serial.print(" Time:");
      Serial.print((uint32_t)emDraw, DEC);
      Serial.print(" writeRect calls:");
      Serial.println(g_WRCount, DEC);
    } else {
      FillScreen(GREEN);
      tft.setTextColor(WHITE);
      tft.setTextSize(2);
      tft.println(F("No Files Found"));
    }
    g_picture_loaded = true;
  }

  //---------------------------------------------------------------------------
  // If the display has a command to update the screen now, see if we should
  // do now or wait until proper time
  //---------------------------------------------------------------------------
  if (g_fast_mode || g_stepMode || (emDisplayed >= (uint32_t)g_display_image_time)) {
    if (g_picture_loaded) {
      tft.updateScreen();
    }
    //---------------------------------------------------------------------------
    // Process any keyboard input.
    //---------------------------------------------------------------------------
    if (g_stepMode) {
      int ch;
      Serial.println("Step Mode: enter anything to continue");
      while ((ch = Serial.read()) == -1) {}  // in case at startup...
      while (ch != -1) {
        if (ch == 'd') g_debug_output = !g_debug_output;
        if (ch == 's') g_stepMode = !g_stepMode;
        if (ch == 'f') g_use_efb = !g_use_efb;
        if (ch == 'l') listFiles();

        ch = Serial.read();
      }
    } else if (Serial.available()) {
      int ch;
      while (Serial.read() != -1)
        ;
      Serial.println("Paused: enter anything to continue");
      while ((ch = Serial.read()) == -1) {}
      while (ch != -1) {
        if (ch == 'd') g_debug_output = !g_debug_output;
        if (ch == 's') g_stepMode = !g_stepMode;
        if (ch == 'f') g_use_efb = !g_use_efb;
        if (ch == 'l') listFiles();
        ch = Serial.read();
      }
    }
    emDisplayed = 0;
    g_picture_loaded = false;
#ifdef TFT_USE_FRAME_BUFFER
  }
#endif
}

int stricmp(const char *s1, const char *s2) {
  while (*s1 != 0 && *s2 != 0) {
    if (*s1 != *s2 && ::toupper(*s1) != ::toupper(*s2)) {
      return -1;
    }
    s1++;
    s2++;
  }
  return (*s1 == 0 && *s2 == 0) ? 0 : -1;
}

//****************************************************************************
// forward function definitions.
//****************************************************************************
inline void writeClippedRect(int16_t x, int16_t y, int16_t cx, int16_t cy, uint16_t *pixels, bool waitForWRC = true) {
  tft.writeRect(x + g_image_offset_x, y + g_image_offset_y, cx, cy, pixels);
}


//=============================================================================
// Options file support - process only if file changed dates (Or first time)
//    example looking for update file.
// This is a real simple parser x=y where x is string y is int...
//=============================================================================
//DateTimeFields g_dtf_optFileLast = { 99 };  // not valid so change first time...
#define MAX_KEY_NAME 20
typedef struct {
  const char key_name[MAX_KEY_NAME];
  int *key_value_addr;
} key_name_value_t;

static const PROGMEM key_name_value_t keyNameValues[] = {
  { "Background", &g_background_color },
  { "debug", &g_debug_output },
  { "Step", &g_stepMode },
  { "BMPScale", &g_BMPScale },
  { "JPGScale", &g_JPGScale },
  { "PNGScale", &g_PNGScale },
  { "ScaleXAbove2", &g_jpg_scale_x_above[SCL_HALF] },
  { "ScaleXAbove4", &g_jpg_scale_x_above[SCL_QUARTER] },
  { "ScaleXAbove8", &g_jpg_scale_x_above[SCL_EIGHTH] },
  { "ScaleXAbove16", &g_jpg_scale_x_above[SCL_16TH] },
  { "ScaleYAbove2", &g_jpg_scale_y_above[SCL_HALF] },
  { "ScaleYAbove4", &g_jpg_scale_y_above[SCL_QUARTER] },
  { "ScaleYAbove8", &g_jpg_scale_y_above[SCL_EIGHTH] },
  { "ScaleYAbove16", &g_jpg_scale_y_above[SCL_16TH] },
  { "Center", &g_center_image },
  { "MaxScaleUp", &g_max_scale_up },
  { "ImageTimeMS", &g_display_image_time }
};

int ReadFileChar(FILE *f) {
  char c;
  size_t ich = fread(&c, 1, 1, f);
  return ich ? c : -1;
}

bool ReadOptionsLine(FILE *optFile, char *key_name, uint8_t sizeof_key, int &key_value) {
  int ch;

  key_value = 0;
  // first lets get key name ignore all whitespace...
  while ((ch = ReadFileChar(optFile)) <= ' ') {
    if (ch < 0) return false;
  }

  uint8_t ich = 0;
  while (ich < (sizeof_key - 1)) {
    if (ch == '=') {
      ch = ReadFileChar(optFile);
      break;
    }
    key_name[ich++] = ch;
    ch = ReadFileChar(optFile);
    if (ch < 0) return false;  //
  }
  key_name[ich] = '\0';

  int sign_value = 1;
  if (ch == '-') {
    sign_value = -1;
    ch = ReadFileChar(optFile);
    if (ch == -1) return false;
  }

  while ((ch >= '0') && (ch <= '9')) {
    key_value = key_value * 10 + ch - '0';
    ch = ReadFileChar(optFile);
  }
  // should probably check for other stuff, but...
  key_value *= sign_value;

  // total hacky but allow hex value
  if ((key_value == 0) && ((ch == 'x') || (ch == 'X'))) {
    ch = ReadFileChar(optFile);
    for (;;) {
      if ((ch >= '0') && (ch <= '9')) key_value = key_value * 16 + ch - '0';
      else if ((ch >= 'a') && (ch <= 'f'))
        key_value = key_value * 16 + 10 + ch - 'a';
      else if ((ch >= 'A') && (ch <= 'F'))
        key_value = key_value * 16 + 10 + ch - 'A';
      else
        break;
      ch = ReadFileChar(optFile);
    }
  }

  return true;
}


bool ProcessOptionsFile(FILE *optfile) {
  if (!optfile) return false;
  int key_value;
  char key_name[20];
#ifdef LATER
  DateTimeFields dtf;
  if (!optfile.getModifyTime(dtf)) return false;
  if (memcmp(&dtf, &g_dtf_optFileLast, sizeof(dtf)) == 0) return false;
  g_dtf_optFileLast = dtf;
  Serial.printf("Updated Options file found date: M: %02u/%02u/%04u %02u:%02u\n",
                dtf.mon + 1, dtf.mday, dtf.year + 1900, dtf.hour, dtf.min);

// do simple scan through file
#endif
  bool found = false;
  while (ReadOptionsLine(optfile, key_name, sizeof(key_name), key_value)) {
    Serial.print("\t>>");
    Serial.print(key_name);
    Serial.print("=");
    Serial.print(key_value, DEC);
    for (uint8_t key_index = 0; key_index < (sizeof(keyNameValues) / sizeof(keyNameValues[0])); key_index++) {
      if (stricmp(key_name, keyNameValues[key_index].key_name) == 0) {
        Serial.print(" was: ");
        Serial.println(*(keyNameValues[key_index].key_value_addr), DEC);
        *(keyNameValues[key_index].key_value_addr) = key_value;
        found = true;
        break;
      }
    }
    if (!found) Serial.println(" ** Unknown Key **");
  }

  return true;
}

void ShowAllOptionValues() {
  Serial.println("\n----------------------------------");
  Serial.print("Sketch uses Option file: ");
  Serial.print(options_file_name);
  Serial.println(" at the root of SD Card");
  Serial.println("\t<All key names>=<current key value");
  for (uint8_t key_index = 0; key_index < (sizeof(keyNameValues) / sizeof(keyNameValues[0])); key_index++) {
    Serial.print("\t");
    Serial.print(keyNameValues[key_index].key_name);
    Serial.print("=");
    Serial.println(*(keyNameValues[key_index].key_value_addr), DEC);
  }
  Serial.println("----------------------------------\n");
}


inline void FillScreen(uint16_t color) {
  tft.fillScreen(color);
}
inline uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) {
  return tft.color565(r, g, b);
}
inline void Color565ToRGB(uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
  tft.color565toRGB(color, r, g, b);
}

//=============================================================================
// BMP support
//=============================================================================
// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance for tiny AVR chips.

#define BUFFPIXEL 80
void bmpDraw(const char *filename, bool fErase) {

  //  FILE     bmpFile;
  int image_width, image_height;        // W+H in pixels
  uint8_t bmpDepth;                     // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;              // Start of image data in file
  uint32_t rowSize;                     // Not always = image_width; may have padding
  uint8_t sdbuffer[3 * BUFFPIXEL];      // pixel buffer (R+G+B per pixel)
  uint16_t buffidx = sizeof(sdbuffer);  // Current position in sdbuffer
  boolean goodBmp = false;              // Set to true on valid header parse
  boolean flip = true;                  // BMP is stored bottom-to-top
  int row, col;
  uint8_t r, g, b;
  uint32_t pos = 0;

  uint16_t *usPixels = nullptr;


  FILE *bmpFile = fopen(filename, "r+");
  if (!bmpFile) {
    Serial.print("Failed to open: ");
    Serial.println(filename);
    return; 
  }

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Parse BMP header
  if (read16(bmpFile) == 0x4D42) {                                   // BMP signature
    uint32_t bmpFileSize __attribute__((unused)) = read32(bmpFile);  // Read & ignore creator bytes
    //Serial.print(F("File size: ")); Serial.println(bmpFileSize);
    (void)read32(bmpFile);             // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile);  // Start of image data
    //Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    uint32_t bmpHdrSize __attribute__((unused)) = read32(bmpFile);
    //Serial.print(F("Header size: ")); Serial.println(bmpHdrSize);
    g_image_width = image_width = read32(bmpFile);
    g_image_height = image_height = read32(bmpFile);

    if (read16(bmpFile) == 1) {    // # planes -- must be '1'
      bmpDepth = read16(bmpFile);  // bits per pixel
      //Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if ((bmpDepth == 24) && (read32(bmpFile) == 0)) {  // 0 = uncompressed

        goodBmp = true;  // Supported BMP format -- proceed!
        Serial.print("Image size: ");
        Serial.print(image_width);
        Serial.print("x");
        Serial.print(image_height);
        Serial.print(" depth:");
        Serial.print(bmpDepth, DEC);
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (image_width * 3 + 3) & ~3;

        // If image_height is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (image_height < 0) {
          image_height = -image_height;
          flip = false;
        }

        g_image_scale = 1;
        g_image_scale_up = 0;
        if (g_BMPScale > 0) {
          g_image_scale = g_BMPScale;  // use what they passed in
        } else if (g_BMPScale < 0) {
          // bugbug experiement - try to scale up...
          if ((image_width * 2 < g_tft_width) && (image_height * 2 < g_tft_height)) {
            // image is less than half the screen...
            // lets try simple scale up...
            g_image_scale_up = g_tft_width / image_width;
            int scale_up_height = g_tft_height / image_height;
            if (scale_up_height < g_image_scale_up) g_image_scale_up = scale_up_height;
            if (g_image_scale_up > g_max_scale_up) g_image_scale_up = g_max_scale_up;

          } else {
            if (image_width > g_tft_width) g_image_scale = (image_width + g_tft_width - 1) / g_tft_width;
            if (image_height > g_tft_height) {
              int yscale = (image_height + g_tft_height - 1) / g_tft_height;
              if (yscale > g_image_scale) g_image_scale = yscale;
            }
          }
        } else {
          if ((image_width > g_jpg_scale_x_above[SCL_16TH]) || (image_height > g_jpg_scale_y_above[SCL_16TH])) {
            g_image_scale = 16;
          } else if ((image_width > g_jpg_scale_x_above[SCL_EIGHTH]) || (image_height > g_jpg_scale_y_above[SCL_EIGHTH])) {
            g_image_scale = 8;
          } else if ((image_width > g_jpg_scale_x_above[SCL_QUARTER]) || (image_height > g_jpg_scale_y_above[SCL_QUARTER])) {
            g_image_scale = 4;
          } else if ((image_width > g_jpg_scale_x_above[SCL_HALF]) || (image_height > g_jpg_scale_y_above[SCL_HALF])) {
            g_image_scale = 2;
          }
        }
        if (g_image_scale_up) {
          if (g_center_image) {
            g_image_offset_x = (g_tft_width - (image_width * g_image_scale_up)) / 2;
            g_image_offset_y = (g_tft_height - (image_height * g_image_scale_up)) / 2;
          } else {
            g_image_offset_x = 0;
            g_image_offset_y = 0;
          }

          g_image_scale = 2;  // bugbug use this to know which row to read in to...
          Serial.print("Scale: ");
          Serial.print(g_image_scale_up, DEC);
          Serial.print(" Image Offsets (");
          Serial.print(g_image_offset_x, DEC);
          Serial.print(", ");
          Serial.print(g_image_offset_y, DEC);
          Serial.println(")");

          if (fErase && (((image_width * g_image_scale_up) < g_tft_width) || ((image_height * g_image_scale_up) < image_height))) {
            FillScreen((uint16_t)g_background_color);
          }

          // now we will allocate large buffer for SCALE*width
          // need 2 rows to work with, and resultant output will be an addition 2x
          usPixels = (uint16_t *)malloc(image_width * (2 + g_image_scale_up * g_image_scale_up) * sizeof(uint16_t));

        } else {
          if (g_center_image) {
            g_image_offset_x = (g_tft_width - (image_width / g_image_scale)) / 2;
            g_image_offset_y = (g_tft_height - (image_height / g_image_scale)) / 2;
          } else {
            g_image_offset_x = 0;
            g_image_offset_y = 0;
          }
          Serial.print("Scale: ");
          Serial.print(g_image_scale, DEC);
          Serial.print(" Image Offsets (");
          Serial.print(g_image_offset_x, DEC);
          Serial.print(", ");
          Serial.print(g_image_offset_y, DEC);
          Serial.println(")");

          if (fErase && (((image_width / g_image_scale) < g_tft_width) || ((image_height / g_image_scale) < image_height))) {
            FillScreen((uint16_t)g_background_color);
          }
          // now we will allocate large buffer for SCALE*width
          usPixels = (uint16_t *)malloc(image_width * g_image_scale * sizeof(uint16_t));
        }

        if (usPixels) {
          for (row = 0; row < image_height; row++) {  // For each scanline...

            // Seek to start of scan line.  It might seem labor-
            // intensive to be doing this on every line, but this
            // method covers a lot of gritty details like cropping
            // and scanline padding.  Also, the seek only takes
            // place if the file position actually needs to change
            // (avoids a lot of cluster math in SD library).
            if (flip)  // Bitmap is stored bottom-to-top order (normal BMP)
              pos = bmpImageoffset + (image_height - 1 - row) * rowSize;
            else  // Bitmap is stored top-to-bottom
              pos = bmpImageoffset + row * rowSize;
            if ((uint32_t)ftell(bmpFile) != pos) {  // Need seek?
              fseek(bmpFile, pos, SEEK_SET);
              buffidx = sizeof(sdbuffer);  // Force buffer reload
            }

            uint16_t *pusRow = usPixels + image_width * (row % g_image_scale);
            for (col = 0; col < image_width; col++) {  // For each pixel...
              // Time to read more pixel data?
              if (buffidx >= sizeof(sdbuffer)) {  // Indeed
                fread(sdbuffer, 1, sizeof(sdbuffer), bmpFile);
                buffidx = 0;  // Set index to beginning
              }

              // Convert pixel from BMP to TFT format, push to display
              b = sdbuffer[buffidx++];
              g = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];
              pusRow[col] = Color565(r, g, b);
            }  // end pixel
            if (g_image_scale_up) {
              ScaleUpWriteClippedRect(row, image_width, usPixels);
            } else if (g_image_scale == 1) {
              writeClippedRect(0, row, image_width, 1, pusRow);
            } else {
              ScaleDownWriteClippedRect(row, image_width, usPixels);
            }
          }                // end scanline
          free(usPixels);  // free it after we are done
          usPixels = nullptr;
        }  // malloc succeeded

      }  // end goodBmp
    }
  }

  fclose(bmpFile);
  if (!goodBmp) Serial.println(F("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(FILE *f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = ReadFileChar(f);  // LSB
  ((uint8_t *)&result)[1] = ReadFileChar(f);  // MSB
  return result;
}

uint32_t read32(FILE *f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = ReadFileChar(f);  // LSB
  ((uint8_t *)&result)[1] = ReadFileChar(f);
  ((uint8_t *)&result)[2] = ReadFileChar(f);
  ((uint8_t *)&result)[3] = ReadFileChar(f);  // MSB
  return result;
}



#if defined(__JPEGDEC__) || defined(__PNGDEC__)
void *myOpen(const char *filename, int32_t *size) {
  myfile = fopen(filename, "r+");
  if (myfile) {
    fseek(myfile, 0, SEEK_END); // seek to end of file
    *size = ftell(myfile); // get current file pointer
    fseek(myfile, 0, SEEK_SET); // seek back to beginning of file
  }
  return &myfile;
}
void myClose(void *handle) {
  if (myfile) fclose(myfile);
}
#endif

//=============================================================================
// TFT Helper functions to work on ILI9341_t3
// which doe snot have offset/clipping support
//=============================================================================

// Function to draw pixels to the display
void ScaleUpWriteClippedRect(int row, int image_width, uint16_t *usPixels) {
  //--------------------------------------------------------------------
  // experiment scale up...
  //--------------------------------------------------------------------
  uint16_t *usRowOut = usPixels + 2 * image_width;  //
  uint8_t r_cur, r_prev, g_cur, g_prev, b_cur, b_prev;
  int red, green, blue;
  int image_width_out = (image_width - 1) * g_image_scale_up + 1;  // we don't fill out the last one.
  // if this is not our first row, then we need to compute the fill in row
  // first...
  // Our buffer has g_image_scale_up rows of data to send in one chunk
  uint16_t *puCol = usRowOut;
  uint16_t *puCurRow;
  uint16_t *puPrevRow;
  if (row & 1) {
    puCurRow = usPixels + image_width;
    puPrevRow = usPixels;
  } else {
    puCurRow = usPixels;
    puPrevRow = usPixels + image_width;
  }

  // First lets generate the one for the actual row;
  uint16_t *p = usRowOut + image_width_out * (g_image_scale_up - 1);
  uint16_t *ppixIn = puCurRow;
  for (int col = 0; col < image_width; col++) {
    // bug bug.. could be faster
    *p = *ppixIn++;  // copy the pixel in to pixel out
    if (col) {
      // Now lets fill in the columns between the prev and new...
      Color565ToRGB(*p, r_cur, g_cur, b_cur);
      Color565ToRGB(*(p - g_image_scale_up), r_prev, g_prev, b_prev);
      for (int j = 1; j < g_image_scale_up; j++) {
        red = (int)r_prev + (((int)r_cur - (int)r_prev) * j) / g_image_scale_up;
        green = (int)g_prev + (((int)g_cur - (int)g_prev) * j) / g_image_scale_up;
        blue = (int)b_prev + (((int)b_cur - (int)b_prev) * j) / g_image_scale_up;
        *(p - g_image_scale_up + j) = Color565(red, green, blue);
      }
    }
    p += g_image_scale_up;
  }

  // except for the first row we now need to fill in the extra rows from the previous one
  if (row) {
    for (int col = 0; col < image_width; col++) {
      Color565ToRGB(*puCurRow++, r_cur, g_cur, b_cur);
      Color565ToRGB(*puPrevRow++, r_prev, g_prev, b_prev);
      for (int i = 1; i < g_image_scale_up; i++) {
        uint16_t *p = puCol + (i - 1) * image_width_out;  // so location for this item
        int red = (int)r_prev + (((int)r_cur - (int)r_prev) * i) / g_image_scale_up;
        int green = (int)g_prev + (((int)g_cur - (int)g_prev) * i) / g_image_scale_up;
        int blue = (int)b_prev + (((int)b_cur - (int)b_prev) * i) / g_image_scale_up;
        *p = Color565(red, green, blue);
        // need to compute middle ones as well.
        if (col) {
          // Now lets fill in the columns between the prev and new...
          Color565ToRGB(*p, r_cur, g_cur, b_cur);
          Color565ToRGB(*(p - g_image_scale_up), r_prev, g_prev, b_prev);
          for (int j = 1; j < g_image_scale_up; j++) {
            red = (int)r_prev + (((int)r_cur - (int)r_prev) * j) / g_image_scale_up;
            green = (int)g_prev + (((int)g_cur - (int)g_prev) * j) / g_image_scale_up;
            blue = (int)b_prev + (((int)b_cur - (int)b_prev) * j) / g_image_scale_up;
            *(p - g_image_scale_up + j) = Color565(red, green, blue);
          }
        }
      }
      puCol += g_image_scale_up;
    }
    writeClippedRect(0, 1 + (row - 1) * g_image_scale_up, image_width_out, g_image_scale_up, usRowOut);
  } else {
    // first row just output it's own data.
    writeClippedRect(0, 0, image_width_out, 1, usRowOut + image_width_out * (g_image_scale_up - 1));
  }
}

void ScaleDownWriteClippedRect(int row, int image_width, uint16_t *usPixels) {
  if ((row % g_image_scale) == (g_image_scale - 1)) {
    //--------------------------------------------------------------------
    // else scale down
    //--------------------------------------------------------------------
    uint16_t newx = 0;
    for (uint16_t pix_cnt = 0; pix_cnt < image_width; pix_cnt += g_image_scale) {
      uint8_t red = 0;
      uint8_t green = 0;
      uint8_t blue = 0;
      float r = 0;
      float g = 0;
      float b = 0;
      for (uint8_t i = 0; i < g_image_scale; i++) {
        for (uint8_t j = 0; j < g_image_scale; j++) {
          Color565ToRGB(usPixels[pix_cnt + i + (j * image_width)], red, green, blue);
          // Sum the squares of components instead
          r += red * red;
          g += green * green;
          b += blue * blue;
        }
      }
      // overwrite the start of our buffer with
      usPixels[newx++] = Color565((uint8_t)sqrt(r / (g_image_scale * g_image_scale)), (uint8_t)sqrt(g / (g_image_scale * g_image_scale)), (uint8_t)sqrt(b / (g_image_scale * g_image_scale)));
    }
    writeClippedRect(0, row / g_image_scale, image_width / g_image_scale, 1, usPixels);
  }
}


//=============================================================================
// JPeg support
//=============================================================================
//used for jpeg files primarily
#ifdef __JPEGDEC__
JPEGDEC jpeg;


void processJPGFile(const char *name, bool fErase) {
  Serial.println();
  Serial.print(F("Loading JPG image '"));
  Serial.print(name);
  Serial.println('\'');
  uint8_t scale = 1;
  if (jpeg.open(name, myOpen, myClose, myReadJPG, mySeekJPG, JPEGDraw)) {
    int image_width = jpeg.getWidth();
    int image_height = jpeg.getHeight();
    int decode_options = 0;
    Serial.print("Image size: ");
    Serial.print(image_width);
    Serial.print("x");
    Serial.print(image_height);
    switch (g_JPGScale) {
      case 1:
        scale = 1;
        decode_options = 0;
        break;
      case 2:
        scale = 2;
        decode_options = JPEG_SCALE_HALF;
        break;
      case 4:
        scale = 4;
        decode_options = JPEG_SCALE_QUARTER;
        break;
      case 8:
        scale = 8;
        decode_options = JPEG_SCALE_EIGHTH;
        break;
      default:
        {
          if ((image_width > g_jpg_scale_x_above[SCL_16TH]) || (image_height > g_jpg_scale_y_above[SCL_16TH])) {
            decode_options = JPEG_SCALE_EIGHTH | JPEG_SCALE_HALF;
            scale = 16;
          } else if ((image_width > g_jpg_scale_x_above[SCL_EIGHTH]) || (image_height > g_jpg_scale_y_above[SCL_EIGHTH])) {
            decode_options = JPEG_SCALE_EIGHTH;
            scale = 8;
          } else if ((image_width > g_jpg_scale_x_above[SCL_QUARTER]) || (image_height > g_jpg_scale_y_above[SCL_QUARTER])) {
            decode_options = JPEG_SCALE_QUARTER;
            scale = 4;
          } else if ((image_width > g_jpg_scale_x_above[SCL_HALF]) || (image_height > g_jpg_scale_y_above[SCL_HALF])) {
            decode_options = JPEG_SCALE_HALF;
            scale = 2;
          }
        }
    }
    if (fErase && ((image_width / scale < g_tft_width) || (image_height / scale < g_tft_height))) {
      FillScreen((uint16_t)g_background_color);
    }

    if (g_center_image) {
      g_image_offset_x = (g_tft_width - image_width / scale) / 2;
      g_image_offset_y = (g_tft_height - image_height / scale) / 2;
    } else {
      g_image_offset_x = 0;
      g_image_offset_y = 0;
    }
    g_image_scale = scale;
    Serial.print("Scale: 1/");
    Serial.print(g_image_scale);
    Serial.print(" Image Offsets (");
    Serial.print(g_image_offset_x);
    Serial.print(", ");
    Serial.print(g_image_offset_y), Serial.println(")");

    jpeg.decode(0, 0, decode_options);
    jpeg.close();
  } else {
    Serial.println("Was not a valid jpeg file");
  }
}


int32_t myReadJPG(JPEGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!myfile) return 0;
  return fread(buffer, 1, length,  myfile);
}
int32_t mySeekJPG(JPEGFILE *handle, int32_t position) {
  if (!myfile) return 0;
  return fseek(myfile, position, SEEK_SET);
}

int JPEGDraw(JPEGDRAW *pDraw) {
  if (g_debug_output) {
    Serial.print("jpeg draw: x,y=");
    Serial.print(pDraw->x); 
    Serial.print(","); 
    Serial.print(pDraw->y);
    Serial.print(", cx,cy = "); 
    Serial.print(pDraw->iWidth); 
    Serial.print(","); 
    Serial.println(pDraw->iHeight);
  }
  writeClippedRect(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
  return 1;
}
#endif

//=============================================================================
// PNG support
//=============================================================================
//used for png files primarily
#ifdef __PNGDEC__
PNG png;

void processPNGFile(const char *name, bool fErase) {
  int rc;

  Serial.println();
  Serial.print(F("Loading PNG image '"));
  Serial.print(name);
  Serial.println('\'');
  rc = png.open((const char *)name, myOpen, myClose, myReadPNG, mySeekPNG, PNGDraw);
  if (rc == PNG_SUCCESS) {
    g_image_width = png.getWidth();
    g_image_height = png.getHeight();
    g_image_scale_up = 0;

    g_image_scale = 1;  // default...
    Serial.print("image specs: ("); Serial.print(g_image_width); Serial.print(" x "); Serial.print(g_image_height); 
    Serial.print("), "); Serial.print(png.getBpp()); Serial.print(" bpp, pixel type: "); Serial.println(png.getPixelType());
    if (g_PNGScale > 0) {
      g_image_scale = g_PNGScale;  // use what they passed in
    } else if (g_PNGScale < 0) {
      if (g_image_width > g_tft_width) g_image_scale = (g_image_width + g_tft_width - 1) / g_tft_width;
      if (g_image_height > g_tft_height) {
        int yscale = (g_image_height + g_tft_height - 1) / g_tft_height;
        if (yscale > g_image_scale) g_image_scale = yscale;
      }
    } else {
      if ((g_image_width > g_jpg_scale_x_above[SCL_16TH]) || (g_image_height > g_jpg_scale_y_above[SCL_16TH])) {
        g_image_scale = 16;
      } else if ((g_image_width > g_jpg_scale_x_above[SCL_EIGHTH]) || (g_image_height > g_jpg_scale_y_above[SCL_EIGHTH])) {
        g_image_scale = 8;
      } else if ((g_image_width > g_jpg_scale_x_above[SCL_QUARTER]) || (g_image_height > g_jpg_scale_y_above[SCL_QUARTER])) {
        g_image_scale = 4;
      } else if ((g_image_width > g_jpg_scale_x_above[SCL_HALF]) || (g_image_height > g_jpg_scale_y_above[SCL_HALF])) {
        g_image_scale = 2;
      }
    }

    if (fErase && (((g_image_width / g_image_scale) < g_tft_width) || ((g_image_height / g_image_scale) < g_image_height))) {
      FillScreen((uint16_t)g_background_color);
    }

    if (g_center_image) {
      g_image_offset_x = (g_tft_width - (png.getWidth() / g_image_scale)) / 2;
      g_image_offset_y = (g_tft_height - (png.getHeight() / g_image_scale)) / 2;
    } else {
      g_image_offset_x = 0;
      g_image_offset_y = 0;
    }

    Serial.print("Scale: 1/"); Serial.print(g_image_scale); 
    Serial.print(" Image Offsets ("); Serial.print(g_image_offset_x); Serial.print(", ");  Serial.print(g_image_offset_y); 
    Serial.println(")");
    uint16_t *usPixels = (uint16_t *)malloc(g_image_width * ((g_image_scale == 1) ? 16 : g_image_scale) * sizeof(uint16_t));
    if (usPixels) {
      rc = png.decode(usPixels, 0);
      png.close();
      free(usPixels);
    } else
      Serial.println("Error could not allocate line buffer");
  } else {
    Serial.print("Was not a valid PNG file RC:"); Serial.println(rc, DEC);
  }
}

int32_t myReadPNG(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!myfile) return 0;
  return fread(buffer, 1, length, myfile);
}
int32_t mySeekPNG(PNGFILE *handle, int32_t position) {
  if (!myfile) return 0;
  return fseek(myfile, position, SEEK_SET);
}

// Function to draw pixels to the display
void PNGDraw(PNGDRAW *pDraw) {
  uint16_t *usPixels = (uint16_t *)pDraw->pUser;
  if (g_image_scale == 1) {
    uint16_t *pusRow = usPixels + pDraw->iWidth * (pDraw->y & 0xf);  // we have 16 lines to work with
    png.getLineAsRGB565(pDraw, pusRow, PNG_RGB565_LITTLE_ENDIAN, 0xffffffff);
    // but we will output 8 lines at time.
    if ((pDraw->y == g_image_height - 1) || ((pDraw->y & 0x7) == 0x7)) {
      //      WaitforWRComplete(); // make sure previous writes are done
      writeClippedRect(0, pDraw->y & 0xfff8, pDraw->iWidth, (pDraw->y & 0x7) + 1,
                       usPixels + (pDraw->y & 0x8) * pDraw->iWidth, false);
    }
  } else {
    uint16_t *pusRow = usPixels + pDraw->iWidth * (pDraw->y % g_image_scale);
    png.getLineAsRGB565(pDraw, pusRow, PNG_RGB565_LITTLE_ENDIAN, 0xffffffff);
    ScaleDownWriteClippedRect(pDraw->y, pDraw->iWidth, usPixels);
  }
}


#endif

//=============================================================================
// Touch screen support
//=============================================================================
#if 0
void ProcessTouchScreen() {
  // See if there's any  touch data for us
  //  if (ts.bufferEmpty()) {
  //    return;
  //  }
  // You can also wait for a touch
  if (!ts.touched()) {
    g_fast_mode = false;
    return;
  }

  // first hack, if screen pressed go very fast
  g_fast_mode = true;

  // Retrieve a point
  TS_Point p = ts.getPoint();

  // p is in ILI9341_t3 setOrientation 1 settings. so we need to map x and y differently.

  Serial.print("X = ");
  Serial.print(p.x);
  Serial.print("\tY = ");
  Serial.print(p.y);
  Serial.print("\tPressure = ");
  Serial.print(p.z);


  // Scale from ~0->4000 to tft.width using the calibration #'s
#if 1  // SCREEN_ORIENTATION_1
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, g_tft_width);
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, g_tft_height);
#else

    uint16_t px = map(p.y, TS_MAXY, TS_MINY, 0, g_tft_width);
    p.y = map(p.x, TS_MINX, TS_MAXX, 0, g_tft_height);
    p.x = px;
#endif
  Serial.print(" (");
  Serial.print(p.x);
  Serial.print(", ");
  Serial.print(p.y);
  Serial.println(")");
}
#endif
char g_path[512];  // should not get this long
void listFiles() {
  strcpy(g_path, "/usb/");
#ifdef LATER
  Serial.print("\n Space Used = ");
  Serial.println(pfs->usedSize());
  Serial.print("Filesystem Size = ");
  Serial.println(pfs->totalSize());
#endif
  printDirectory(0);  // we are at the root directory.
}

void printDirectory(int numSpaces) {
  DIR *d = opendir(g_path);
  if (!d) return;
  struct dirent *dir_entry;
  if (numSpaces > 0) strcat(g_path, "/");  // if not root append /
  int path_len = strlen(g_path);
  while ((dir_entry = readdir(d)) != nullptr) {
    printSpaces(numSpaces);
    Serial.print(dir_entry->d_name);
    if (dir_entry->d_type == DT_DIR) {
      Serial.println("/");
      strcpy(&g_path[path_len], dir_entry->d_name);
      printDirectory(numSpaces + 2);
    } else {
      // files have sizes, directories do not
      Serial.println();
    }
  }
  closedir(d);
}

void printSpaces(int num) {
  for (int i = 0; i < num; i++) {
    Serial.print(" ");
  }
}
