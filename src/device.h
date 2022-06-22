//
// Created by robosam2003 on 22/06/2022.
//

#ifndef ARDUINO_LSM6DS032_DEVICE_H
#define ARDUINO_LSM6DS032_DEVICE_H

#include "Wire.h"


class device { // abstract class with the aim of allowing the same operations for both SPI and I2C (and possibly other) protocols
public:
    virtual void device_begin(uint32_t freq) = 0; // begin device at desired frequency - This could indicate BAUD for UART device ???

    virtual byte read_reg(byte regAddress) = 0;
    virtual void read_regs(byte regAddress, byte* outputPointer, uint length) = 0; // function overload

    virtual bool write_reg(byte regAddress, byte data) = 0;
    virtual bool write_regs(byte regAddress ,byte* writeBuffer, uint length) = 0;

    ~device() = default; // destructor
};


class I2CDevice : public device {
protected:
    byte deviceAddress;
    TwoWire *pipe;
public:
    I2CDevice(byte i2c_address, TwoWire* i2c_pipe) : deviceAddress {i2c_address}, pipe {i2c_pipe} {} // constructor

    void device_begin(uint32_t freq) override;
    byte read_reg(byte regAddress) override;
    void read_regs(byte regAddress, byte* outputPointer,  uint length) override;
    bool write_reg(byte regAddress, byte data) override;
    bool write_regs(byte regAddress ,byte* writeBuffer, uint length) override;

    ~I2CDevice() {pipe->end();}; // destructor - For if you want to control the device's lifetime

};

#endif //ARDUINO_LSM6DS032_DEVICE_H
