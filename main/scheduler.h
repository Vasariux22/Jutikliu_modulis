#pragma once
#include <vector>
#include <cstdint> 
#include "ppg_basic.h"
#include "imu.h"
#include "temp.h"
#include "menu.h"

#define MAX_PPG_SAMPLES 110
#define MAX_IMU_SAMPLES 30
#define MAX_TEMP_SAMPLES 4

extern float tempC;

void SensorScheduler_init(TwoWire* ppgBus, uint32_t  ppgHz, uint16_t  imuHz, uint16_t  tempHz);

void SensorScheduler_deinit();