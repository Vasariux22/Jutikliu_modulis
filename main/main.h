/** main.h
*/
#pragma once
#include <stdint.h>

#define PWR_EN  7
#define PB_OK   8
#define PB_UP   15
#define PB_DOWN 16
#define CRG_MODE 48

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define SDA_PIN 40
#define SCL_PIN 41

// USB MUX
#define USB_MUX  42
#define SD_EN 45

#define ESP32_SD_D0  13  //SPI_MISO
#define ESP32_SD_D1  9
#define ESP32_SD_D2  14 
#define ESP32_SD_D3  10 //SPI_CS
#define ESP32_SD_CMD 11 //SPI_MOSI
#define ESP32_SD_CLK 12 //SPI_CLK

