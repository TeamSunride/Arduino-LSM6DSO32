//
// Created by robos on 23/06/2022.
//
#include "LSM6DS032.h"


LSM6DS032::LSM6DS032(TwoWire *pipe, uint32_t freq) {
    device = new I2CProtocol(LSM6DS032_DEFAULT_I2C_ADDRESS, pipe, freq);
}

LSM6DS032::LSM6DS032(byte chipSelect, SPIClass& spi, SPISettings settings) {
    device = new SPIProtocol(chipSelect, spi, settings, READ_BYTE, WRITE_BYTE);
}