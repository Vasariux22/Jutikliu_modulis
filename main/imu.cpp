#include "imu.h"
#include "LSM6DSOSensor.h"
#include "lsm6dso_reg.h"
#include <stdio.h>

static LSM6DSOSensor* imu = nullptr;
static bool acc_enabled = false;
static bool gyr_enabled = false;

void imu_gpio_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << IMU_INT1_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE 
    };
    gpio_config(&io_conf);
}

void imu_init(TwoWire* wire, const IMUUserConfig& config) {
    static LSM6DSOSensor instance(wire, LSM6DSO_I2C_ADD_L);
    imu = &instance;

    if (imu->begin() != 0) {
        printf("IMU init failed\n");
        return;
    }

    imu->Enable_X();
    acc_enabled = true;
    imu->Enable_G();
    gyr_enabled = true;

    imu->Write_Reg(0x07, 0x40);

    imu->Set_X_ODR(static_cast<float>(config.sampleRateHz));
    imu->Set_G_ODR(static_cast<float>(config.sampleRateHz));

    imu->Set_X_FS(static_cast<float>(config.accelScale));
    imu->Set_G_FS(static_cast<float>(config.gyroScale));

    imu->Set_FIFO_X_BDR(config.sampleRateHz);
    imu->Set_FIFO_G_BDR(config.sampleRateHz);

    imu->Set_FIFO_Mode(LSM6DSO_STREAM_MODE);
    imu->Set_FIFO_Watermark_Level(52);
    imu->Set_FIFO_Stop_On_Fth(0);
    imu->Set_FIFO_INT1_FIFO_Full(1); 


    printf("IMU initialized with config: ODR=%dHz, Accel=%dg, Gyro=%ddps\n",
           config.sampleRateHz, config.accelScale, config.gyroScale);
}

void imu_read(int32_t acc[3], int32_t gyr[3]) {
    if (!imu) {
        printf("IMU not initialized\n");
        return;
    }

    imu->Get_X_Axes(acc);
    imu->Get_G_Axes(gyr);
}

int imu_read_fifo(IMUSample* buffer, size_t maxSamples) {
    if (!imu || !buffer || maxSamples == 0)
        return 0;

    uint16_t samples = 0;
    if (imu->Get_FIFO_Num_Samples(&samples) != LSM6DSO_OK || samples < 1)
        return 0;

    size_t readCount = 0;
    int32_t acc[3] = {0}, gyr[3] = {0};
    bool accReady = false, gyrReady = false;

    while (samples-- > 0 && readCount < maxSamples) {
        uint8_t tag;
        if (imu->Get_FIFO_Tag(&tag) != LSM6DSO_OK)
            break;

        uint8_t type = tag & 0x1F;

        if (type == 0x02) { // Accelerometer
            if (imu->Get_FIFO_X_Axes(acc) == LSM6DSO_OK)
                accReady = true;
        } else if (type == 0x01) { // Gyroscope
            if (imu->Get_FIFO_G_Axes(gyr) == LSM6DSO_OK)
                gyrReady = true;
        } else {
            uint8_t dummy[6];
            imu->Get_FIFO_Data(dummy);
        }

        if (accReady && gyrReady) {
            memcpy(buffer[readCount].acc, acc, sizeof(acc));
            memcpy(buffer[readCount].gyr, gyr, sizeof(gyr));
            readCount++;
            accReady = false;
            gyrReady = false;
        }
    }
    return readCount;
}


void imu_disable() {
    if (!imu) return;

    imu->Disable_X();
    acc_enabled = false;

    imu->Disable_G();
    gyr_enabled = false;
}


void imu_enable_acc() {
    if (imu && imu->Enable_X() == 0)
        acc_enabled = true;
}

void imu_enable_gyro() {
    if (imu && imu->Enable_G() == 0)
        gyr_enabled = true;
}


void imu_reset_fifo() {
    if (!imu) return;
    imu->Set_FIFO_Mode(LSM6DSO_BYPASS_MODE);
    imu->Set_FIFO_Mode(LSM6DSO_STREAM_MODE);
}
