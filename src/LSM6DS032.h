//
// Created by robos on 23/06/2022.
//

#ifndef ARDUINO_LSM6DS032_LSMDS032_H
#define ARDUINO_LSM6DS032_LSMDS032_H

#include "protocol.h"

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
*/

// bit 0: RW bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the device is read. In latter case, the chip will drive SDO at the start of bit 8. ** Datasheet: 5.1.2, pg. 37 **
#define WRITE_BYTE 0b00000000
#define READ_BYTE 0b10000000

#define LSM6DS032_DEFAULT_I2C_ADDRESS 0x6A // from back of adafruit breakout board


class LSM6DS032 {
protected:
    protocol* device;
public:
    LSM6DS032(TwoWire *pipe, uint32_t freq); // constructor
    LSM6DS032(byte chipSelect, SPIClass& spi, SPISettings settings); // constructor

    void LSM6DS032_begin() {
        device->protocol_begin();
        // any other set up etc
    }
    void foo() {}

};



#endif //ARDUINO_LSM6DS032_LSMDS032_H
