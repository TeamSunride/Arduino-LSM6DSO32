//
// Created by robosam2003 on 22/06/2022.
//

#ifndef ARDUINO_LSM6DS032_DEVICE_H
#define ARDUINO_LSM6DS032_DEVICE_H

#include "Wire.h"
#include "SPI.h"



class protocol { // abstract class with the aim of allowing the same operations for both SPI and I2C (and possibly other) protocols
public:
    virtual void protocol_begin() = 0 ; // begin protocol

    virtual byte read_reg(byte regAddress) = 0;
    virtual void read_regs(byte regAddress, byte* outputPointer, uint length) = 0;

    virtual bool write_reg(byte regAddress, byte data) = 0;
    virtual bool write_regs(byte regAddress ,byte* writeBuffer, uint length) = 0;

    ~protocol() = default; // destructor
};


class I2CProtocol : public protocol {
protected:
    byte _deviceAddress;
    TwoWire *_pipe;
    uint32_t _freq;
public:
    I2CProtocol(byte i2c_address, TwoWire* i2c_pipe, uint32_t freq); // constructor defined elsewhere

    void protocol_begin() override;
    byte read_reg(byte regAddress) override;
    void read_regs(byte regAddress, byte* outputPointer,  uint length) override;
    bool write_reg(byte regAddress, byte data) override;
    bool write_regs(byte regAddress ,byte* writeBuffer, uint length) override;

    ~I2CProtocol() { _pipe->end(); }; // destructor - For if you want to control the protocol's lifetime

};


class SPIProtocol : public protocol {
protected:
    byte CS; // chip select pin used
    SPIClass _spi;
    SPISettings _settings;
    byte READ_BYTE; // these are unique to every device - check the datasheet
    byte WRITE_BYTE;
public:
    SPIProtocol(byte chipSelect, SPIClass spiChannel, SPISettings settings, byte READ, byte WRITE); // constructor defined elsewhere


    void protocol_begin() override;
    byte read_reg(byte regAddress) override;
    void read_regs(byte regAddress, byte* outputPointer,  uint length) override;
    bool write_reg(byte regAddress, byte data) override;
    bool write_regs(byte regAddress ,byte* writeBuffer, uint length) override;

    ~SPIProtocol() { _spi.end(); }; // destructor

};

#endif //ARDUINO_LSM6DS032_DEVICE_H
