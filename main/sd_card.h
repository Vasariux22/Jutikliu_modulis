#pragma once

#include "esp_err.h"
#include "temp.h"
#include "imu.h"
#include "ppg_basic.h"


esp_err_t sd_card_init(void);
void startNewSDLogFile();
void stopSDLogFile();
void sd_card_write_samples(size_t pc, size_t ic, size_t tc,const PPGSample* ppg, const IMUSample* imu, const TempSample* temp);


