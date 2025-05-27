#pragma once
#include <Arduino.h>
#include <Wire.h>

struct PPGUserConfig {
    uint8_t sampleRateReg;
    bool ledEnabled[4];
    uint8_t ledWidth; 

    uint8_t ledPower[4];
    uint8_t ledRange[4];
};

struct PPGSample {
    uint32_t ir;
    uint32_t red;
    uint32_t green;
    uint32_t blue;
};

void ppg_basic_init(TwoWire* wire, const PPGUserConfig& config);
bool ppg_basic_read(TwoWire* wire, uint32_t* ir, uint32_t* red, uint32_t* green, uint32_t* blue);
void ppg_basic_disable(TwoWire* wire);

