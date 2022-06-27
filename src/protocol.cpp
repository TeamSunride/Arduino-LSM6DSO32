//
// Created by robosam2003 on 22/06/2022.
//
#include "protocol.h"


/* Bit manipulation */
byte getBit(byte bits, int bitIndex) {
    return (bits & (1 << (bitIndex))) ? 1 : 0;
}

void setBit(byte* bits, int bitIndex, int val) { // index starts from lsb (at 0) and goes left
    // This way, bits is ORed a one that is shifted left by bitIndex, which sets the bit to 1
    // likewise bit is ANDed with a byte like 0b11101111 for example, if the bit index was 4. This sets ONLY the 5th bit to 0;
    (val) ? (*bits |= (1 << (bitIndex))) : (*bits &= ~(1 << (bitIndex)));
}





// I2CProtocol code written by Tom Danvers in https://github.com/TeamSunride/Arduino-BNO055
I2CProtocol::I2CProtocol(byte i2c_address, TwoWire *i2c_pipe, uint32_t freq) {
    _deviceAddress = i2c_address;
    _pipe = i2c_pipe;
    _freq = freq;
}


uint8_t I2CProtocol::protocol_begin()  {
    _pipe->begin(); // by passing _pipe, it allows the user to use Wire1, Wire2 etc.
    _pipe->setClock(_freq);
    return 0; // zero for success - not applicable to all protocols
}

byte I2CProtocol::read_reg(byte regAddress) {
    _pipe->beginTransmission(_deviceAddress);
    _pipe->write(regAddress);
    _pipe->endTransmission();

    byte numBytes = 1;
    byte result = 0;
    _pipe->requestFrom(_deviceAddress, numBytes);
    while (_pipe->available()) {
        result = _pipe->read();
    }
    return result;
}

void I2CProtocol::read_regs(byte regAddress, byte* outputPointer,  uint length)  {
    uint8_t externalSpacePointer = 0;
    char c = 0;
    _pipe->beginTransmission(_deviceAddress);
    _pipe->write(regAddress);
    _pipe->endTransmission();

    _pipe->requestFrom(_deviceAddress, length);
    while (_pipe->available() && (externalSpacePointer < length)) {
        c = _pipe->read();
        *outputPointer = c;
        outputPointer++;
        externalSpacePointer++;
    }
}


uint8_t I2CProtocol::write_reg(byte regAddress, byte data) {
    _pipe->beginTransmission(_deviceAddress);
    _pipe->write(regAddress);
    _pipe->write(data);
    return _pipe->endTransmission() ; // 0 means success
}

uint8_t I2CProtocol::write_regs(byte regAddress ,byte* writeBuffer, uint length) {
    _pipe->beginTransmission(_deviceAddress);
    _pipe->write(regAddress);

    byte bytesWritten = 0;
    while (bytesWritten < length) {
        _pipe->write(*writeBuffer);
        writeBuffer++;
        bytesWritten++;
    }
    return _pipe->endTransmission(); // 0 means success

}




// **** SPIProtocol ****
SPIProtocol::SPIProtocol(byte chipSelect, SPIClass spiChannel, SPISettings settings, byte READ, byte WRITE) : _spi {spiChannel} // _spi must be constructed explicitly as there is no default constructor for that class
{
    CS = chipSelect;
    _settings = settings;
    READ_BYTE = READ;
    WRITE_BYTE = WRITE;
}



uint8_t SPIProtocol::protocol_begin() {
    pinMode(CS, OUTPUT); // Other pins are configured on spi.begin().
    _spi.beginTransaction(_settings);
    _spi.begin(); // you have to begin the SPI bus before you can use it --_--
    return 0; // 0 for success - not applicable to all protocols
}

byte SPIProtocol::read_reg(byte regAddress) {
    regAddress = READ_BYTE | regAddress; // puts read bit into the 8th bit.
    byte inByte = 0;
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    _spi.transfer(regAddress);  // transfers address over MOSI line
    inByte = _spi.transfer(0x00);
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
    return inByte;

}

void SPIProtocol::read_regs(byte regAddress, byte *outputPointer, uint length) {
    regAddress = READ_BYTE | regAddress; // puts read bit into the 8th bit.
    byte inByte = 0;
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    _spi.transfer(regAddress); // transfer address of desired register
    for (uint i=0; i<length; i++) {
        inByte = _spi.transfer(0x00);  // transfers 0x00 over MOSI line, recieves a byte over MISO line.
        *outputPointer = inByte;
        outputPointer++; // increment output pointer.
    }
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
}

uint8_t SPIProtocol::write_reg(byte regAddress, byte data) {
    regAddress = WRITE_BYTE | regAddress; //
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    _spi.transfer(regAddress);
    _spi.transfer(data);
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
    return 0; // 0 for success - not applicable to all protocols
}

uint8_t SPIProtocol::write_regs(byte regAddress, byte *writeBuffer, uint length) {
    regAddress = WRITE_BYTE | regAddress;
    digitalWrite(CS, LOW); // pulls CS low, which begins the transfer
    _spi.transfer(regAddress);
    for (uint i=0; i<length; i++) {
        _spi.transfer(*writeBuffer);
        writeBuffer++; // increment writeBuffer pointer
    }
    digitalWrite(CS, HIGH); // pulls CS high, which ends the transfer
    return 0; // 0 for success - not applicable to all protocols
}

