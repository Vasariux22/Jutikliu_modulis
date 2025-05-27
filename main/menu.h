#pragma once
#include <stdint.h>
#include "ppg_basic.h" 
#include "imu.h"
#include "main.h"

extern uint16_t desiredRate;
extern uint16_t desiredIMURate;
extern uint8_t activePPGChannels;
extern bool wifiEnabled;
extern bool sdEnabled;
extern PPGUserConfig latestPPGConfig;

void menu_update();
void drawMenu();
bool menu_isReady();
void enterMenu();

PPGUserConfig getDefaultPPGConfig();
PPGUserConfig menu_getConfig();

IMUUserConfig getDefaultIMUConfig();
IMUUserConfig menu_getIMUConfig();
