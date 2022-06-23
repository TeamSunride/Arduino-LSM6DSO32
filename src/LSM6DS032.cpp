//
// Created by robos on 23/06/2022.
//
#include "LSM6DS032.h"


LSM6DS032::LSM6DS032(TwoWire *pipe, uint32_t freq) { // constructor for I2C protocol
    device = new I2CProtocol(LSM6DS032_DEFAULT_I2C_ADDRESS, pipe, freq);
}

LSM6DS032::LSM6DS032(byte chipSelect, SPIClass& spi, SPISettings settings) { // constructor for SPI protocol
    // TODO: is there a way to assert/ensure that the SPI settings are that of the LSM6DS032? - namely, MSB first, SPI mode 2.
    device = new SPIProtocol(chipSelect, spi, settings, READ_BYTE, WRITE_BYTE);
}

// Cheers GitHub copilot ;)

void LSM6DS032::enable_embedded_functions(bool enable) {
    byte reg = device->read_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS);
    reg = reg & 0b11000000; // last 6 bits ' must be set to '0' for the correct operation of the device. ' according to the datasheet
    (enable) ? setBit(&reg, 7, 1) : setBit(&reg, 7, 0);
    device->write_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS, reg);
}

void LSM6DS032::enable_sensor_hub(bool enable) {
    byte reg = device->read_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS);
    reg = reg & 0b11000000; // last 6 bits ' must be set to '0' for the correct operation of the device. ' according to the datasheet
    (enable) ? setBit(&reg, 6, 1) : setBit(&reg, 6, 0);
    device->write_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS, reg);
}

void LSM6DS032::enable_sdo_pullup(bool enable) {
    byte reg = device->read_reg(LSM6DS032_REGISTER::PIN_CTRL);
    setBit(&reg, 7, 0);  reg = reg & 0b11000000; // essential for operation of the device
    (enable) ? setBit(&reg, 6, 1) : setBit(&reg, 6, 0);
    device->write_reg(LSM6DS032_REGISTER::PIN_CTRL, reg);
}
