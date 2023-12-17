/*!
* @file ILI9488.cpp
*
* @mainpage ILI9488 TFT Displays
*
* @section intro_sec Introduction
*
* This is the documentation for ILI9488 driver for the Arduino platform.
*
* These displays use SPI to communicate, 4 or 5 pins are required
* to interface (RST is optional).
*
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
* @section dependencies Dependencies
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* @section author Author
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* @section license License
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include "ILI9488.h"
#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
  #ifndef RASPI
    #include "wiring_private.h"
  #endif
#endif
#include <limits.h>

#if defined (ARDUINO_ARCH_ARC32) || defined (ARDUINO_MAXIM)
  #define SPI_DEFAULT_FREQ  16000000
// Teensy 3.0, 3.1/3.2, 3.5, 3.6
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #define SPI_DEFAULT_FREQ  40000000
#elif defined (__AVR__) || defined(TEENSYDUINO)
  #define SPI_DEFAULT_FREQ  8000000
#elif defined(ESP8266) || defined(ESP32)
  #define SPI_DEFAULT_FREQ  40000000
#elif defined(RASPI)
  #define SPI_DEFAULT_FREQ  80000000
#elif defined(ARDUINO_ARCH_STM32F1)
  #define SPI_DEFAULT_FREQ  36000000
#else
  #define SPI_DEFAULT_FREQ  24000000  ///< Default SPI data clock frequency
#endif

#if defined (ARDUINO_ARCH_SPRESENSE)
  #define SPI_MODE SPI_MODE3
#else
  #define SPI_MODE SPI_MODE0
#endif

#define MADCTL_MY  0x80  ///< Bottom to top
#define MADCTL_MX  0x40  ///< Right to left
#define MADCTL_MV  0x20  ///< Reverse Mode
#define MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04  ///< LCD refresh right to left

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9488 driver with software SPI
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    mosi  SPI MOSI pin #
    @param    sclk  SPI Clock pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
    @param    miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
ILI9488::ILI9488(int8_t cs, int8_t dc, int8_t mosi,int8_t sclk, int8_t rst, int8_t miso) :
  Adafruit_SPITFT(ILI9488_TFTWIDTH, ILI9488_TFTHEIGHT, cs, dc, mosi, sclk, rst, miso) {
}

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9488 driver with hardware SPI using the
            default SPI peripheral.
    @param  cs   Chip select pin # (OK to pass -1 if CS tied to GND).
    @param  dc   Data/Command pin # (required).
    @param  rst  Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
ILI9488::ILI9488(int8_t cs, int8_t dc, int8_t rst) :
  Adafruit_SPITFT(ILI9488_TFTWIDTH, ILI9488_TFTHEIGHT, cs, dc, rst) {
}

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9488 driver with hardware SPI using
            a specific SPI peripheral (not necessarily default).
    @param  spiClass  Pointer to SPI peripheral (e.g. &SPI or &SPI1).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and
                      CS is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
ILI9488::ILI9488(SPIClass *spiClass, int8_t dc, int8_t cs, int8_t rst) :
  Adafruit_SPITFT(ILI9488_TFTWIDTH, ILI9488_TFTHEIGHT, spiClass, cs, dc, rst) {
}
#endif // end !ESP8266

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9488 driver using parallel interface.
    @param  busWidth  If tft16 (enumeration in Adafruit_SPITFT.h), is a
                      16-bit interface, else 8-bit.
    @param  d0        Data pin 0 (MUST be a byte- or word-aligned LSB of a
                      PORT register -- pins 1-n are extrapolated from this).
    @param  wr        Write strobe pin # (required).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and CS
                      is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
    @param  rd        Read strobe pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
ILI9488::ILI9488(tftBusWidth busWidth, int8_t d0, int8_t wr, int8_t dc, int8_t cs,
  int8_t rst, int8_t rd) : Adafruit_SPITFT(ILI9488_TFTWIDTH, ILI9488_TFTHEIGHT, busWidth,
    d0, wr, dc, cs, rst, rd) {
}

static const uint8_t PROGMEM initcmd[] = {
  ILI9488_PWCTR1, 2, 0x17,0x15,
  ILI9488_PWCTR2, 1, 0x41,
  ILI9488_VMCTR1, 3, 0x00,0x12,0x80,
  ILI9488_MADCTL, 1, 0x40,
  ILI9488_PIXFMT, 1, 0x66,
  ILI9488_FRMCTR1, 1, 0xA0,
  ILI9488_INVCTR, 1, 0x02,
  ILI9488_DFUNCTR, 2, 0x02,0x02,
  0xF7, 4, 0xA9,0x51,0x2C,0x82,
  ILI9488_SLPOUT  , 0x80,                // Exit Sleep
  ILI9488_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};

/**************************************************************************/
/*!
    @brief   Initialize ILI9488 chip
    Connects to the ILI9488 over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
void ILI9488::begin(uint32_t freq) {

    if(!freq) freq = SPI_DEFAULT_FREQ;
    initSPI(freq, SPI_MODE);

    if(_rst < 0) {                     // If no hardware reset pin...
        sendCommand(ILI9488_SWRESET); // Engage software reset
        delay(150);
    }

    uint8_t        cmd, x, numArgs;
    const uint8_t *addr = initcmd;
    while((cmd = pgm_read_byte(addr++)) > 0) {
        x = pgm_read_byte(addr++);
        numArgs = x & 0x7F;
        sendCommand(cmd, addr, numArgs);
        addr += numArgs;
        if(x & 0x80) delay(150);
    }

    _width  = ILI9488_TFTWIDTH;
    _height = ILI9488_TFTHEIGHT;
}

/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void ILI9488::setRotation(uint8_t m) {
    rotation = m % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            m = (MADCTL_MX);
            _width  = ILI9488_TFTWIDTH;
            _height = ILI9488_TFTHEIGHT;
            break;
        case 1:
            m = (MADCTL_MV);
            _width  = ILI9488_TFTHEIGHT;
            _height = ILI9488_TFTWIDTH;
            break;
        case 2:
            m = (MADCTL_MY);
            _width  = ILI9488_TFTWIDTH;
            _height = ILI9488_TFTHEIGHT;
            break;
        case 3:
            m = (MADCTL_MX | MADCTL_MY | MADCTL_MV);
            _width  = ILI9488_TFTHEIGHT;
            _height = ILI9488_TFTWIDTH;
            break;
    }

    sendCommand(ILI9488_MADCTL, &m, 1);
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void ILI9488::invertDisplay(bool invert) {
    sendCommand(invert ? ILI9488_INVON : ILI9488_INVOFF);
}

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void ILI9488::scrollTo(uint16_t y) {
    uint8_t data[2];
    data[0] = y >> 8;
    data[1] = y & 0xff;
    sendCommand(ILI9488_VSCRSADD, (uint8_t*) data, 2);
}

/**************************************************************************/
/*!
    @brief   Set the height of the Top and Bottom Scroll Margins
    @param   top The height of the Top scroll margin
    @param   bottom The height of the Bottom scroll margin
 */
/**************************************************************************/
void ILI9488::setScrollMargins(uint16_t top, uint16_t bottom) {
    if (top + bottom <= ILI9488_TFTHEIGHT) {
        uint16_t middle = ILI9488_TFTHEIGHT - top + bottom;
        uint8_t data[6];
        data[0] = top >> 8;
        data[1] = top & 0xff;
        data[2] = middle >> 8;
        data[3] = middle & 0xff;
        data[4] = bottom >> 8;
        data[5] = bottom & 0xff;
        sendCommand(ILI9488_VSCRDEF, (uint8_t*) data, 6);
    }
}

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM with the next chunk of
             SPI data writes. The ILI9488 will automatically wrap the data as each row is filled
    @param   x1  TFT memory 'x' origin
    @param   y1  TFT memory 'y' origin
    @param   w   Width of rectangle
    @param   h   Height of rectangle
*/
/**************************************************************************/
void ILI9488::setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
    uint16_t x2 = (x1 + w - 1),
             y2 = (y1 + h - 1);
    writeCommand(ILI9488_CASET); // Column address set
    SPI_WRITE16(x1);
    SPI_WRITE16(x2);
    writeCommand(ILI9488_PASET); // Row address set
    SPI_WRITE16(y1);
    SPI_WRITE16(y2);
    writeCommand(ILI9488_RAMWR); // Write to RAM
}

/**************************************************************************/
/*!
    @brief  Read 8 bits of data from ILI9488 configuration memory. NOT from RAM!
            This is highly undocumented/supported, it's really a hack but kinda works?
    @param    commandByte  The command register to read data from
    @param    index  The byte index into the command to read from
    @return   Unsigned 8-bit data read from ILI9488 register
 */
/**************************************************************************/
uint8_t ILI9488::readcommand8(uint8_t commandByte, uint8_t index) {
    uint8_t data = 0x10 + index;
    sendCommand(0xD9, &data, 1); // Set Index Register
    return Adafruit_SPITFT::readcommand8(commandByte);
}

/**************************************************************************/
void ILI9488::writePixel(int16_t x, int16_t y, uint16_t color) {
    if((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
        setAddrWindow(x, y, 1, 1);
        uint8_t r = color & 0x1f;
        uint8_t g = (color >> 5) & 0x3f;
        uint8_t b = (color >> 11) & 0x1f;
        spiWrite(r<<3);
        spiWrite(g<<2);
        spiWrite(b<<3);
    }
}

/**************************************************************************/
void ILI9488::writePixels(uint8_t *colors, uint32_t len, bool block, bool bigEndian) {

    if(!len) return; // Avoid 0-byte transfers

    hwspi._spi->send(colors, len);

}

/**************************************************************************/
void ILI9488::writeColor(uint16_t color, uint32_t len) {

    len = len * 3;

    #define SPI_MAX_PIXELS_AT_ONCE (512*3)
    static uint8_t temp[SPI_MAX_PIXELS_AT_ONCE];
    uint16_t       bufLen = (len < SPI_MAX_PIXELS_AT_ONCE) ? len : SPI_MAX_PIXELS_AT_ONCE, 
                   xferLen;

    for(uint32_t t=0; t < bufLen; t=t+3) {
        temp[t]   = (color & 0x1f) << 3;         // blue
        temp[t+1] = ((color >> 5) & 0x3f) << 2;  // green
        temp[t+2] = ((color >> 11) & 0x1f) << 3; // red
    }

    while(len) {                               // While pixels remain
        xferLen = (bufLen < len) ? bufLen : len; // How many this pass?
        writePixels(temp, xferLen);
       len -= xferLen;
    }
}
