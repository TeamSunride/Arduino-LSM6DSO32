//
// Created by robosam2003 on 23/06/2022.
// TeamSunride
#ifndef ARDUINO_LSM6DS032_LSMDS032_H
#define ARDUINO_LSM6DS032_LSMDS032_H

#include "protocol.h"
#include "LSM6DS032_registers.h"
/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
*/


/* Datasheet: 5.1.2, pg. 37
 bit 0: RW bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the
 device is read. In latter case, the chip will drive SDO at the start of bit 8.
*/
#define WRITE_BYTE 0b00000000
#define READ_BYTE 0b10000000

#define LSM6DS032_DEFAULT_I2C_ADDRESS 0x6A // from back of adafruit breakout board


/*
 * The LSM6DSO32 has three operating modes available:
 * - only accelerometer active and gyroscope in power-down
 * - only gyroscope active and accelerometer in power-down
 * - both accelerometer and gyroscope sensors active with independent ODR
 */
enum LSM6DS032_OPERATING_MODES {
    ACCEL_ONLY,
    GYRO_ONLY,
    ACCEL_AND_GYRO // Independent Output Data Rates (ODR)
};


enum LSM6DS032_ACCEL_POWER_MODES {
    ACCEL_POWER_DOWN,
    ACCEL_ULTRA_LOW_POWER,
    ACCEL_LOW_POWER,
    ACCEL_NORMAL,
    ACCEL_HIGH_PERFORMANCE
};

enum LSM6DS032_GYRO_POWER_MODES {
    GYRO_POWER_DOWN,
    GYRO_ULTRA_LOW_POWER,
    GYRO_LOW_POWER,
    GYRO_NORMAL,
    GYRO_HIGH_PERFORMANCE
};


class LSM6DS032 { // This could maybe be a child of the IMU class??
protected:
    protocol* device;
public:
    LSM6DS032(TwoWire *pipe, uint32_t freq); // constructor overload for I2C protocol
    LSM6DS032(byte chipSelect, SPIClass& spi, SPISettings settings); // constructor overload for SPI protocol

    void begin() {
        device->protocol_begin();
        // any other set up etc
    }

    void enable_embedded_functions(bool enable);
    void enable_sensor_hub(bool enable);
    void enable_sdo_pullup(bool enable);

};



#endif //ARDUINO_LSM6DS032_LSMDS032_H
