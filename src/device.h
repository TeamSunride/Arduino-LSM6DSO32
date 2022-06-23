//
// Created by robosam2003 on 22/06/2022.
//

#ifndef ARDUINO_LSM6DS032_DEVICE_H
#define ARDUINO_LSM6DS032_DEVICE_H

#include "Wire.h"
#include "SPI.h"

enum SPI_NUM {
    SPI_CHANNEL_0 = 0, // just to make things more readable when initialising SPI
    SPI_CHANNEL_1 = 1,
    SPI_CHANNEL_2 = 2
};

#define SPI0_MOSI_PIN 11
#define SPI0_MISO_PIN 12
#define SPI0_SCK_PIN 13

#define SPI1_MOSI_PIN 26
#define SPI1_MISO_PIN 1
#define SPI1_SCK_PIN 27

// On teensy 4.1, SPI2 may be used for SD card or QSPI flash - see https://www.pjrc.com/store/teensy41_card11b_rev4.png for alternative pin assignments.
#define SPI2_MOSI_PIN 43
#define SPI2_MISO_PIN 44
#define SPI2_SCK_PIN 45

class device { // abstract class with the aim of allowing the same operations for both SPI and I2C (and possibly other) protocols
public:
    virtual void device_begin(uint32_t freq) = 0; // begin device at desired frequency - This could indicate BAUD for UART device ???

    virtual byte read_reg(byte regAddress) = 0;
    virtual void read_regs(byte regAddress, byte* outputPointer, uint length) = 0;

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


class SPIDevice : public device {
protected:
    byte CS; // chip select pin used
    byte SBFIRST;
    byte SPIMODE;
    SPI_NUM SPINUM; // the spi channel used, SPI0, SPI1 etc
    SPIClass spi;
    byte READ_BYTE; // these are unique to every device - check the datasheet
    byte WRITE_BYTE;
public:
    SPIDevice(byte chipSelect, byte sbFirst, byte spiMode, SPI_NUM spiNum, byte READ, byte WRITE)  // constructor
        : CS {chipSelect}, SBFIRST {sbFirst}, SPIMODE {spiMode}, SPINUM {spiNum},
        spi {SPI}, READ_BYTE {READ}, WRITE_BYTE {WRITE} {} // defaults to SPI0 but will be changed in device_begin() as needed

    void device_begin(uint32_t freq) override;
    byte read_reg(byte regAddress) override;
    void read_regs(byte regAddress, byte* outputPointer,  uint length) override;
    bool write_reg(byte regAddress, byte data) override;
    bool write_regs(byte regAddress ,byte* writeBuffer, uint length) override;

    ~SPIDevice() { // destructor
        switch (SPINUM) {
            case(0): {SPI.end(); break; }
            case(1): {SPI1.end(); break; }
            case(2): {SPI2.end(); break; }
        }
    }
};

#endif //ARDUINO_LSM6DS032_DEVICE_H
