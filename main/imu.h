#pragma once
#include <Wire.h>
#include <stdint.h>

#define IMU_INT1_GPIO GPIO_NUM_39

struct IMUUserConfig {
    uint16_t sampleRateHz;
    uint8_t accelScale;
    uint16_t gyroScale;
};

struct __attribute__((packed)) IMUSample {
    int32_t acc[3];
    int32_t gyr[3];
};

void imu_init(TwoWire* wire, const IMUUserConfig& config);
void imu_read(int32_t acc[3], int32_t gyr[3]);
void imu_disable();
int imu_read_fifo(IMUSample* buffer, size_t maxSamples);
void imu_reset_fifo();
void imu_enable_acc();
void imu_enable_gyro();