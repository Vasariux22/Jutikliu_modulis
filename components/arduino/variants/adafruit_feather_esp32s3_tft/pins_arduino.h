#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>
#include "soc/soc_caps.h"

#define USB_VID          0x239A
#define USB_PID          0x811D
#define USB_MANUFACTURER "Adafruit"
#define USB_PRODUCT      "Feather ESP32-S3 TFT"
#define USB_SERIAL       ""  // Empty string for MAC address

// User LED
#define LED_BUILTIN 13
#define BUILTIN_LED LED_BUILTIN  // backward compatibility

// Neopixel
#define PIN_NEOPIXEL 33
// RGB_BUILTIN and RGB_BRIGHTNESS can be used in new Arduino API rgbLedWrite() and digitalWrite() for blinking
#define RGB_BUILTIN    (PIN_NEOPIXEL + SOC_GPIO_PIN_COUNT)
#define RGB_BRIGHTNESS 64

#define NEOPIXEL_NUM      1     // number of neopixels
#define NEOPIXEL_POWER    34    // power pin
#define NEOPIXEL_POWER_ON HIGH  // power pin state when on

#define TFT_I2C_POWER 21
#define TFT_CS        7
#define TFT_RST       40
#define TFT_DC        39
#define TFT_BACKLITE  45

static const uint8_t SDA = 42;
static const uint8_t SCL = 41;

static const uint8_t SS = 7;
static const uint8_t MOSI = 35;
static const uint8_t SCK = 36;
static const uint8_t MISO = 37;

static const uint8_t A0 = 18;
static const uint8_t A1 = 17;
static const uint8_t A2 = 16;
static const uint8_t A3 = 15;
static const uint8_t A4 = 14;
static const uint8_t A5 = 8;

static const uint8_t TX = 1;
static const uint8_t RX = 2;
#define TX1 TX
#define RX1 RX

static const uint8_t T1 = 1;
static const uint8_t T2 = 2;
static const uint8_t T5 = 5;
static const uint8_t T6 = 6;
static const uint8_t T8 = 8;
static const uint8_t T9 = 9;
static const uint8_t T10 = 10;
static const uint8_t T11 = 11;
static const uint8_t T12 = 12;
static const uint8_t T13 = 13;
static const uint8_t T14 = 14;

static const uint8_t DAC1 = 17;
static const uint8_t DAC2 = 18;

#endif /* Pins_Arduino_h */
