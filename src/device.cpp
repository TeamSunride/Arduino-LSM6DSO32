//
// Created by robosam2003 on 22/06/2022.
//
#include "device.h"

// I2CDevice code written by Tom Danvers in https://github.com/TeamSunride/Arduino-BNO055
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
    pipe->beginTransmission(deviceAddress);
    pipe->write(regAddress);
    pipe->write(data);
    return pipe->endTransmission() == 0; // 0 means success
}

bool I2CDevice::write_regs(byte regAddress ,byte* writeBuffer, uint length) {
    pipe->beginTransmission(deviceAddress);
    pipe->write(regAddress);

    byte bytesWritten = 0;
    while (bytesWritten < length) {
        pipe->write(*writeBuffer);
        writeBuffer++;
        bytesWritten++;
    }
    return pipe->endTransmission() == 0; // 0 means success

}




// **** SPIDevice ****
void SPIDevice::device_begin(uint32_t freq) {
    pinMode(CS, OUTPUT); // CS pin is pulled low when device is selected
    switch (SPINUM) {
        case(0): {
            spi = SPI; // the spi channel is determined by SPINUM
            pinMode(SPI0_MOSI_PIN, OUTPUT);
            pinMode(SPI0_MISO_PIN, INPUT);
            pinMode(SPI0_SCK_PIN, OUTPUT);
            break;
        }
        case(1): {
            spi = SPI1;
            pinMode(SPI1_MOSI_PIN, OUTPUT);
            pinMode(SPI1_MISO_PIN, INPUT);
            pinMode(SPI1_SCK_PIN, OUTPUT);
            break;
        }
        case(2): {
            spi = SPI2;
            pinMode(SPI2_MOSI_PIN, OUTPUT);
            pinMode(SPI2_MISO_PIN, INPUT);
            pinMode(SPI2_SCK_PIN, OUTPUT);
            break;
        }
        default:{ // defaults to SPI0
            spi = SPI;
            pinMode(SPI0_MOSI_PIN, OUTPUT);
            pinMode(SPI0_MISO_PIN, INPUT);
            pinMode(SPI0_SCK_PIN, OUTPUT);
            break;
        }
    }
    spi.beginTransaction(SPISettings(freq, SBFIRST, SPIMODE));
    spi.begin(); // you have to begin to

}

byte SPIDevice::read_reg(byte regAddress) {
    regAddress = READ_BYTE | regAddress; // puts read bit into the 8th bit.
    byte inByte = 0;
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    spi.transfer(regAddress);  // transfers address over MOSI line
    inByte = spi.transfer(0x00);
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
    return inByte;

}

void SPIDevice::read_regs(byte regAddress, byte *outputPointer, uint length) {
    regAddress = READ_BYTE | regAddress; // puts read bit into the 8th bit.
    byte inByte = 0;
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    spi.transfer(regAddress); // transfer address of desired register
    for (uint i=0; i<length; i++) {
        inByte = spi.transfer(0x00);  // transfers 0x00 over MOSI line, recieves a byte over MISO line.
        *outputPointer = inByte;
        outputPointer++; // increment output pointer.
    }
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
}

bool SPIDevice::write_reg(byte regAddress, byte data) {
    regAddress = WRITE_BYTE | regAddress; //
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    spi.transfer(regAddress);
    spi.transfer(data);
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
    return true;
}

bool SPIDevice::write_regs(byte regAddress, byte *writeBuffer, uint length) {
    regAddress = WRITE_BYTE | regAddress;
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    spi.transfer(regAddress);
    for (uint i=0; i<length; i++) {
        spi.transfer(*writeBuffer);
        writeBuffer++; // increment writeBuffer pointer
    }
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
    return true;
}
