//
// Created by robos on 23/06/2022.
//

#ifndef ARDUINO_LSM6DS032_LSMDS032_H
#define ARDUINO_LSM6DS032_LSMDS032_H

#include "device.h"

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
*/

#define WRITE_BYTE 0b00000000 // bit 0: RW bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the device is read. In latter case, the chip will drive SDO at the start of bit 8.
#define READ_BYTE 0b10000000

#define LSM6DS032_DEFAULT_I2C_ADDRESS 0x6A


enum deviceSelect {
    I2C_SELECT,
    SPI_SELECT
};


class LSM6DS032 {
protected:
    device* sensor;
public:
    LSM6DS032(deviceSelect I2C_OR_SPI, byte chipSelect, SPI_NUM spiNum); // constructor

    void foo() {}

};

LSM6DS032::LSM6DS032(deviceSelect I2C_OR_SPI, byte chipSelect, SPI_NUM spiNum) { // constructor
    switch (I2C_OR_SPI) {
        case(I2C_SELECT): { I2CDevice d(LSM6DS032_DEFAULT_I2C_ADDRESS, &Wire); sensor = &d; break;}
        case(SPI_SELECT): { SPIDevice d(chipSelect, MSBFIRST, SPI_MODE2, spiNum, READ_BYTE, WRITE_BYTE); sensor = &d; break;}
    }
}


#endif //ARDUINO_LSM6DS032_LSMDS032_H
