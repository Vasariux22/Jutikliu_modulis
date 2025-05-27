#include "temp.h"

static MLX90632 mlx;

void temp_init(TwoWire* wire) {
    MLX90632::status tempStatus;
    if (!mlx.begin(MLX90632_DEFAULT_ADDRESS, *wire, tempStatus)) {    
        Serial.println("MLX90632 init not ok");
    } else {
        Serial.println("MLX90632 init ok");
        mlx.continuousMode(); 
    }
}

float temp_get() {
    return mlx.getObjectTemp(); 
}

void temp_disable() {
    mlx.sleepMode();
}

void temp_trigger() {
    mlx.setSOC(); 
}

