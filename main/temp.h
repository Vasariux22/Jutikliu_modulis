#pragma once
#include <Wire.h>
#include "MLX90632.h"

struct __attribute__((packed)) TempSample {
    float temperatureC;
};

void temp_init(TwoWire* wire);
float temp_get();
void temp_disable();
void temp_trigger();
