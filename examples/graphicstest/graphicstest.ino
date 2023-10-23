#include <LibPrintf.h>

#include <ILI9341_GIGA_n.h>
// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_RST 8
#define TFT_CS 10

ILI9341_GIGA_n tft(&SPI1, TFT_CS, TFT_DC, TFT_RST);

void setup() {
  // put your setup code here, to run once:
  while (!Serial && millis() < 5000) {}
  Serial.begin(115200);
  Serial.println("Before tft.begin");
  delay(50);
  tft.begin();
  Serial.println("after tft.begin");
  printSPIRegisters();
  delay(50);
}

void printSPIRegisters() {
  if (tft._pgigaSpi) {
    printf("\nHardware SPI registers\n");
    printf("\tCR1:  %x\n", tft._pgigaSpi->CR1);         /*!< SPI/I2S Control register 1,                      Address offset: 0x00 */
    printf("\tCR2:  %x\n", tft._pgigaSpi->CR2);         /*!< SPI Control register 2,                          Address offset: 0x04 */
    printf("\tCFG1:  %x\n", tft._pgigaSpi->CFG1);       /*!< SPI Configuration register 1,                    Address offset: 0x08 */
    printf("\tCFG2:  %x\n", tft._pgigaSpi->CFG2);       /*!< SPI Configuration register 2,                    Address offset: 0x0C */
    printf("\tIER:  %x\n", tft._pgigaSpi->IER);         /*!< SPI/I2S Interrupt Enable register,               Address offset: 0x10 */
    printf("\tSR:  %x\n", tft._pgigaSpi->SR);           /*!< SPI/I2S Status register,                         Address offset: 0x14 */
    printf("\tIFCR:  %x\n", tft._pgigaSpi->IFCR);       /*!< SPI/I2S Interrupt/Status flags clear register,   Address offset: 0x18 */
    printf("\tTXDR:  %x\n", tft._pgigaSpi->TXDR);       /*!< SPI/I2S Transmit data register,                  Address offset: 0x20 */
    printf("\tRXDR:  %x\n", tft._pgigaSpi->RXDR);       /*!< SPI/I2S Receive data register,                   Address offset: 0x30 */
    printf("\tCRCPOLY:  %x\n", tft._pgigaSpi->CRCPOLY); /*!< SPI CRC Polynomial register,                     Address offset: 0x40 */
    printf("\tTXCRC:  %x\n", tft._pgigaSpi->TXCRC);     /*!< SPI Transmitter CRC register,                    Address offset: 0x44 */
    printf("\tRXCRC:  %x\n", tft._pgigaSpi->RXCRC);     /*!< SPI Receiver CRC register,                       Address offset: 0x48 */
    printf("\tUDRDR:  %x\n", tft._pgigaSpi->UDRDR);     /*!< SPI Underrun data register,                      Address offset: 0x4C */
    printf("\tI2SCFGR:  %x\n", tft._pgigaSpi->I2SCFGR); /*!< I2S Configuration register,                      Address offset: 0x50 */
  }
}

void loop() {
  delay(50);
  // put your main code here, to run repeatedly:
  // Init the cam QVGA, using 15fps instead of 30fps to get better refresh rate
  uint32_t start_time = millis();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.fillScreen(ILI9341_RED);
  delay(500);
  yield();
  tft.fillScreen(ILI9341_GREEN);
  yield();
  tft.fillScreen(ILI9341_BLUE);
  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  Serial.println(millis() - start_time, DEC);
  delay(500);
  if (Serial.available()) {
    printSPIRegisters();
    while (Serial.read() != -1)
      ;
  }
}
