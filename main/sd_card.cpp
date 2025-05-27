#include "sd_card.h"
#include "esp_log.h"
#include <stdio.h>
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "menu.h"
#include "esp_timer.h"
#include <dirent.h>
#include <string.h>

static sdmmc_card_t* card;
static FILE* log_file = NULL;
static int file_index = -1; 

esp_err_t sd_card_init(void) {
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_host_device_t spi_host = SPI2_HOST;
    host.slot = spi_host;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_10;
    slot_config.host_id = spi_host;

    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE("SD_SPI", "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    DIR* dir = opendir("/sdcard");
    int maxIndex = 0;

    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL) {
            int index = 0;
            if (strncasecmp(entry->d_name, "LOG_", 4) == 0 && strcasestr(entry->d_name, ".CSV")) {
                if (sscanf(entry->d_name, "LOG_%d.CSV", &index) == 1 && index >= 0) {
                    if (index >= maxIndex) maxIndex = index + 1;
                }
            }
        }
        closedir(dir);
        file_index = maxIndex;
    } else {
        file_index = 0;
    }

    return ESP_OK;
}

void startNewSDLogFile() {
    if (log_file) fclose(log_file);

    char filename[64];
    snprintf(filename, sizeof(filename), "/sdcard/log_%03d.csv", file_index++);
    log_file = fopen(filename, "w");

    if (!log_file) {
        ESP_LOGE("SD_LOG", "Failed to create %s", filename);
        sdEnabled = false;
        return;
    }

    ESP_LOGI("SD_LOG", "Logging to: %s", filename);
    fprintf(log_file, "timestamp_ms");

    if (activePPGChannels & (1 << 0)) fprintf(log_file, ",ir[]");
    if (activePPGChannels & (1 << 1)) fprintf(log_file, ",red[]");
    if (activePPGChannels & (1 << 2)) fprintf(log_file, ",green[]");
    if (activePPGChannels & (1 << 3)) fprintf(log_file, ",blue[]");

    fprintf(log_file, ",acc[],gyro[],temp[]\n");

    fflush(log_file);
    sdEnabled = true;
}


void stopSDLogFile() {
    if (log_file) {
        fclose(log_file);
        log_file = nullptr;
    }

    esp_vfs_fat_sdcard_unmount("/sdcard", card);
    card = NULL;
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(USB_MUX, LOW);
    sdEnabled = false;
}

void sd_card_write_samples(size_t pc, size_t ic, size_t tc,
    const PPGSample* ppg, const IMUSample* imu, const TempSample* temp) {
    
    if (!log_file) return;

    uint64_t timestamp_ms = esp_timer_get_time() / 1000;
    fprintf(log_file, "%llu", timestamp_ms);

    // ---- PPG Channels ----
    if (activePPGChannels & (1 << 0)) {
        fprintf(log_file, ",\"[");
        for (size_t i = 0; i < pc; ++i) {
            fprintf(log_file, "%lu", ppg[i].ir);
            if (i < pc - 1) fprintf(log_file, ",");
        }
        fprintf(log_file, "]\"");
    }

    if (activePPGChannels & (1 << 1)) {
        fprintf(log_file, ",\"[");
        for (size_t i = 0; i < pc; ++i) {
            fprintf(log_file, "%lu", ppg[i].red);
            if (i < pc - 1) fprintf(log_file, ",");
        }
        fprintf(log_file, "]\"");
    }

    if (activePPGChannels & (1 << 2)) {
        fprintf(log_file, ",\"[");
        for (size_t i = 0; i < pc; ++i) {
            fprintf(log_file, "%lu", ppg[i].green);
            if (i < pc - 1) fprintf(log_file, ",");
        }
        fprintf(log_file, "]\"");
    }

    if (activePPGChannels & (1 << 3)) {
        fprintf(log_file, ",\"[");
        for (size_t i = 0; i < pc; ++i) {
            fprintf(log_file, "%lu", ppg[i].blue);
            if (i < pc - 1) fprintf(log_file, ",");
        }
        fprintf(log_file, "]\"");
    }

    // ---- ACC ----
    fprintf(log_file, ",\"[");
    for (size_t i = 0; i < ic; ++i) {
        fprintf(log_file, "[%ld,%ld,%ld]", imu[i].acc[0], imu[i].acc[1], imu[i].acc[2]);
        if (i < ic - 1) fprintf(log_file, ",");
    }
    fprintf(log_file, "]\"");

    // ---- GYRO ----
    fprintf(log_file, ",\"[");
    for (size_t i = 0; i < ic; ++i) {
        fprintf(log_file, "[%ld,%ld,%ld]", imu[i].gyr[0], imu[i].gyr[1], imu[i].gyr[2]);
        if (i < ic - 1) fprintf(log_file, ",");
    }
    fprintf(log_file, "]\"");

    // ---- TEMP ----
    fprintf(log_file, ",\"[");
    for (size_t i = 0; i < tc; ++i) {
        fprintf(log_file, "%.2f", temp[i].temperatureC);
        if (i < tc - 1) fprintf(log_file, ",");
    }
    fprintf(log_file, "]\"\n");

    fflush(log_file);
}

