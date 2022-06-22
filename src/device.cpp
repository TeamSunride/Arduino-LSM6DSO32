//
// Created by robosam2003 on 22/06/2022.
//
#include "device.h"


void I2CDevice::device_begin(uint32_t freq)  {
    pipe->begin(); // by passing pipe, it allows the user to use Wire1, Wire2 etc.
    pipe->setClock(freq);
}

byte I2CDevice::read_reg(byte regAddress) {
    pipe->beginTransmission(deviceAddress);
    pipe->write(regAddress);
    pipe->endTransmission();

    byte numBytes = 1;
    byte result = 0;
    pipe->requestFrom(deviceAddress, numBytes);
    while (pipe->available()) {
        result = pipe->read();
    }
    return result;
}

void I2CDevice::read_regs(byte regAddress, byte* outputPointer,  uint length)  {
    uint8_t externalSpacePointer = 0;
    char c = 0;
    pipe->beginTransmission(deviceAddress);
    pipe->write(regAddress);
    pipe->endTransmission();

    pipe->requestFrom(deviceAddress, length);
    while (pipe->available() && (externalSpacePointer < length)) {
        c = pipe->read();
        *outputPointer = c;
        outputPointer++;
        externalSpacePointer++;
    }
}


bool I2CDevice::write_reg(byte regAddress, byte data) {

}

bool I2CDevice::write_regs(byte regAddress ,byte* writeBuffer, uint length) {

}


