#include "ppg_basic.h"

#define MAX86916_I2C_ADDR 0x57
#define REG_INT_ENABLE        0x02
#define REG_FIFO_WR_PTR       0x04
#define REG_FIFO_OVF_CNT      0x05
#define REG_FIFO_RD_PTR       0x06
#define REG_FIFO_DATA         0x07
#define REG_FIFO_CONFIG       0x08
#define REG_MODE_CONFIG1      0x09
#define REG_MODE_CONFIG2      0x0A
#define REG_LED1_PA           0x0C
#define REG_LED2_PA           0x0D
#define REG_LED3_PA           0x0E
#define REG_LED4_PA           0x0F
#define REG_LED_SEQ1          0x13
#define REG_LED_SEQ2          0x14
#define REG_LED_RANGE         0x11

const uint8_t ledSlotCodes[4] = {0x01, 0x02, 0x03, 0x04};
static uint8_t activeSlots[4];
static uint8_t numActiveSlots = 0;


static void writeReg(TwoWire* wire, uint8_t reg, uint8_t val) {
    wire->beginTransmission(MAX86916_I2C_ADDR);
    wire->write(reg);
    wire->write(val);
    wire->endTransmission();
}

static bool readReg(TwoWire* wire, uint8_t reg, uint8_t* buf, uint8_t len) {
    wire->beginTransmission(MAX86916_I2C_ADDR);
    wire->write(reg);
    if (wire->endTransmission() != 0) return false;

    wire->requestFrom(MAX86916_I2C_ADDR, len);
    for (uint8_t i = 0; i < len; ++i) {
        if (!wire->available()) return false;
        buf[i] = wire->read();
    }
    return true;
}

void setLEDRanges(TwoWire* wire, const uint8_t range[4]) {
    uint8_t regValue = 0;
    regValue |= (range[0] & 0x03) << 0;
    regValue |= (range[1] & 0x03) << 2;
    regValue |= (range[2] & 0x03) << 4;
    regValue |= (range[3] & 0x03) << 6;

    writeReg(wire, REG_LED_RANGE, regValue);
}

void ppg_basic_init(TwoWire* wire, const PPGUserConfig& cfg) {
    writeReg(wire, REG_MODE_CONFIG1, 0x03);

    uint8_t modeCfg2 =
    ((3 & 0x03) << 5) | 
    ((cfg.sampleRateReg & 0x07) << 2) |
    (cfg.ledWidth & 0x03);

    writeReg(wire, REG_MODE_CONFIG2, modeCfg2);

    uint8_t seq1 = 0;
    uint8_t seq2 = 0;
    uint8_t slot = 0;

    for (uint8_t i = 0; i < 4; i++) {
        if (cfg.ledEnabled[i]) {
            uint8_t code = ledSlotCodes[i];
            if (slot == 0) {
                seq1 |= (code & 0x0F);
            } else if (slot == 1) {
                seq1 |= (code & 0x0F) << 4;
            } else if (slot == 2) {
                seq2 |= (code & 0x0F);
            } else if (slot == 3) {
                seq2 |= (code & 0x0F) << 4;
            }
            slot++;
        }
    }
    numActiveSlots = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (cfg.ledEnabled[i]) {
            activeSlots[numActiveSlots++] = i;
        }
    }

    writeReg(wire, REG_LED_SEQ1, seq1);
    writeReg(wire, REG_LED_SEQ2, seq2);

    for (uint8_t i = 0; i < 4; i++) {
        writeReg(wire, REG_LED1_PA + i, cfg.ledPower[i]);
    }

    setLEDRanges(wire, cfg.ledRange);

    writeReg(wire, REG_FIFO_CONFIG, 0x10);
    writeReg(wire, REG_INT_ENABLE, 0x00);

    writeReg(wire, REG_FIFO_WR_PTR, 0x00);
    writeReg(wire, REG_FIFO_RD_PTR, 0x00);
    writeReg(wire, REG_FIFO_OVF_CNT, 0x00);
}

bool ppg_basic_read(TwoWire* wire, uint32_t* ir, uint32_t* red, uint32_t* green, uint32_t* blue) {
    uint8_t raw[12]; 
    uint8_t numBytes = 3 * numActiveSlots;

    if (!readReg(wire, REG_FIFO_DATA, raw, numBytes)) return false;

    *ir = *red = *green = *blue = 0;

    for (uint8_t s = 0; s < numActiveSlots; s++) {
        uint32_t value = ((raw[s*3] << 16) | (raw[s*3 + 1] << 8) | raw[s*3 + 2]) & 0x7FFFF;
        switch (activeSlots[s]) {
            case 0: *ir = value; break;
            case 1: *red = value; break;
            case 2: *green = value; break;
            case 3: *blue = value; break;
        }
    }

    return true;
}

void ppg_basic_disable(TwoWire* wire) {
    writeReg(wire, REG_MODE_CONFIG1, 0x00);
}

