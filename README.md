Overview and Warning: 
=====
** NEEDS EDITING WIP MAY NEVER GET MUCH FARTHER ***
This Arduino library is for driving ILI9341 displays on an Arduino GIGA R1 board. I
 have played with it on a few different
ILI9341 displays including ones from PJRC such as: https://www.pjrc.com/store/display_ili9341_touch.html and ones from Adafruit such as: https://www.adafruit.com/product/1770

This is a port of my ILI9341_t3n library which was earlier derived from te official PJRC ILI9341_t3 library (https://github.com/PaulStoffregen/ILI9341_t3).

And it is always a Work In Progress.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


This library borrows some concepts and functionality like the usage of DMA from another variant library: https://github.com/FrankBoesing/ILI9341_t3DMA

Constructor and begin
----
This library was developed to allow you to use any of the SPI busses on a Teensy 3.x or 4.x processor. 
It detects this by looking at which pins were specified on the constructor. 

```
  //   pspi: either  &SPI (6 pin spi connector) or &SPI1 (shield pins)
  //   CS: Chip select pin,  DC: Data/Command pin
  //   RST: optional reset pin
  //   dmaStream: If using DMA which dma stream to use (DMA[12]_Stream[0-7])
  ILI9341_GIGA_n(SPIClass *pspi, uint8_t _CS, uint8_t _DC, uint8_t _RST = 255, 
                DMA_Stream_TypeDef * dmaStream = DMA1_Stream1);

  // Constructor
  //   CS: Chip select pin,  DC: Data/Command pin
  //   RST: optional reset pin
  //   MOSI, SCLK, MISO: I do nothing with these, don't think there are alternatives
  ILI9341_GIGA_n(uint8_t CS, uint8_t DC, uint8_t RST = 255, uint8_t MOSI = 11,
  ILI9341_GIGA_n(uint8_t _CS, uint8_t _DC, uint8_t _RST = 255, uint8_t _MOSI = 11,
```

When the begin is called, if the constructor you called did not specify an SPI buss,
unlike the teensy version, it assumes SPI object and the MISO/MOSI/SCK are ignored. 
```
  void begin(uint32_t spi_clock = ILI9341_SPICLOCK,
             uint32_t spi_clock_read = ILI9341_SPICLOCK_READ);
```

You are free to use any digital pin for CS and DC.

Frame Buffer
------------
I borrowed some ideas from the ILI9341_t3DMA library and added code to be able to use a logical Frame Buffer.  
To enable this I added a couple of API's 
```c++
    uint8_t useFrameBuffer(boolean b) - if b non-zero it will allocate memory and start using
    void	freeFrameBuffer(void) - Will free up the memory that was used.
    void	updateScreen(void); - Will update the screen with all of your updates...
	void	setFrameBuffer(uint16_t *frame_buffer); - Now have the ability allocate the frame buffer and pass it in, to avoid use of malloc
```
Asynchronous Update support (Frame buffer)
------------------------

The code now has support to use DMA for Asynchronous updates of the screen.  You can choose to do the updates once or in continuous mode.  Note: I mainly use the 
oneshot as I prefer more control on when the screen updates which helps to minimize things like flashing and tearing. 
Some of the New methods for this include: 

```c++
	bool	updateScreenAsync(bool update_cont = false); - Starts an update either one shot or continuous
	void	waitUpdateAsyncComplete(void);  - Wait for any active update to complete
	void	endUpdateAsync();			 - Turn of the continuous mode.
	boolean	asyncUpdateActive(void)      - Lets you know if an async operation is still active
```

Additional APIs
---------------
In addition, this library now has some of the API's and functionality that has been requested in a pull request.  In particular it now supports, the ability to set a clipping rectangle as well as setting an origin that is used with the drawing primitives.   These new API's include:
```c++
	void setOrigin(int16_t x = 0, int16_t y = 0); 
	void getOrigin(int16_t* x, int16_t* y);
	void setClipRect(int16_t x1, int16_t y1, int16_t w, int16_t h); 
	void setClipRect();
```

This library borrows some concepts and functionality from other libraries as well, such as: from the TFT_ILI9341_ESP, https://github.com/Bodmer/TFT_ILI9341_ESP, for additional functions:
```c++
    int16_t  drawNumber(long long_num,int poX, int poY);
    int16_t  drawFloat(float floatNumber,int decimal,int poX, int poY);   
    int16_t drawString(const String& string, int poX, int poY);
    int16_t drawString(char string[], int16_t len, int poX, int poY);
    void setTextDatum(uint8_t datum);
```

In addition, scrolling text has been added using appropriate function from, https://github.com/vitormhenrique/ILI9341_t3:
```c++
    void enableScroll(void);
    void resetScrollBackgroundColor(uint16_t color);
    void setScrollTextArea(int16_t x, int16_t y, int16_t w, int16_t h);
    void setScrollBackgroundColor(uint16_t color);
    void scrollTextArea(uint8_t scrollSize);
    void resetScrollBackgroundColor(uint16_t color);
```

Some other member functions have been added by request, that have not been fully tested, nor will they work with the
frame buffer.
```c++
	void setScrollMargins(uint16_t top, uint16_t bottom);  // Note this is now also member of Adafruit library
```

Font Support
------------
This library tries to support three different font types.  This includes the original font support that is in the ILI9341_t3 library, which is 
built in system font, as well as the new font format. 

In addition, we added support to use the Adafruit GFX fonts as well. This includes the ability to output the text in Opaque mode. 

The text output support also has been exteneded in a way similar to the RA8875 library to allow for easier centering of text. 

The member function setCursor has been extended in a couple of ways:
```c++
	void setCursor(int16_t x, int16_t y, bool autoCenter=false);
```
if the autoCenter is true, the next text output will be centered at the given x, y location.  Note: this is only true for the NEXT output.  

In addition you can pass in the magic value: ILI9341_t3n::CENTER for x and/or y and the next text output will be centered horizontally and/or vertically centered in the screen. 

Some of the example sketches, such as the ILI_Ada_FontTest3 and ILI_Ada_FontTest4 may require an additional library of fonts to work.
You can find this font library up at: https://github.com/mjs513/ILI9341_fonts
This font library is setup, to create an archive file of all of the fonts, such that it won't pull in every font file in this directory, 
unlike some other libraries.  Likewise it is setup to hopefully work with several of our libraries by conditionally including header files
to try to match which display library you are using. 



Discussion regarding this optimized version:
==========================

https://forum.arduino.cc/t/adafruit-ili9341-and-spi-library-issues-on-giga-board/1179783

The Teensy version which this is ported from:
http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-%28320x240-TFT-color-display%29-library

This version of the library supports the Adafruit displays that use the ILI9341 displays, but in
addition are setup to support the displays that are sold by PJRC, which include:
	http://pjrc.com/store/display_ili9341.html
	http://pjrc.com/store/display_ili9341_touch.html

Note: this library like the ILI9341_t3 library which it is derived from no longer  require any of the Adafruit libraries, such as their Adafruit_ILI9341 and Adafruit_GFX libraries APIS are based on.

Adafruit library info
=======================

But as this code is based of of their work, their original information is included below:

------------------------------------------

This is a library for the Adafruit ILI9341 display products

This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
  ----> http://www.adafruit.com/products/1651
 
Check out the links above for our tutorials and wiring diagrams.
These displays use SPI to communicate, 4 or 5 pins are required
to interface (RST is optional).

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
MIT license, all text above must be included in any redistribution

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder Adafruit_ILI9341. Check that the Adafruit_ILI9341 folder contains Adafruit_ILI9341.cpp and Adafruit_ILI9341.

Place the Adafruit_ILI9341 library folder your arduinosketchfolder/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE

Also requires the Adafruit_GFX library for Arduino.

Future Updates
==============


Again WIP
=====
