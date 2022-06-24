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

enum BATCHING_DATA_RATES { /// most are common across accelerometer and gyro except for 1.6 Hz and 6.5Hz
    /** Selects Batching Data Rate (writing frequency in FIFO) for gyroscope / accelerometer data.
        (0000: not batched in FIFO (default);
    */
    NO_BATCHING = 0b0000,
    BDR_12_5Hz = 0b0001,
    BDR_26Hz = 0b0010,
    BDR_52Hz = 0b0011,
    BDR_104Hz = 0b0100,
    BDR_208Hz = 0b0101,
    BDR_417Hz = 0b0110,
    BDR_833Hz = 0b0111,
    BDR_1667Hz = 0b1000,
    BDR_3333Hz = 0b1001,
    BDR_6667Hz = 0b1010,
    BDR_XL_1_6Hz = 0b1011, // NOTE: this batching rate is only for the accelerometer
    BDR_GY_6_5Hz = 0b1011  // NOTE: this batching rate is only for the gyroscope
};

enum TIMESTAMP_BATCHING_DECIMATION {
    /** Selects decimation for timestamp batching in FIFO. Writing rate will be the maximum
        rate between XL and GYRO BDR divided by decimation decoder. */
    DECIMATION_1 = 0b01, /// max(BDR_XL[Hz],BDR_GY[Hz]) [Hz];
    DECIMATION_8 = 0b10, /// max(BDR_XL[Hz],BDR_GY[Hz])/8 [Hz];
    DECIMATION_32 = 0b11 /// max(BDR_XL[Hz],BDR_GY[Hz])/32 [Hz];
};

enum TEMPERATURE_BATCHING_RATE {
    /** Selects batching data rate (writing frequency in FIFO) for temperature data
        (00: Temperature not batched in FIFO (default); */
    ODR_T_1_6_Hz = 0b01,
    ODR_T_12_5_Hz = 0b10,
    ODR_T_52_Hz = 0b11
};

enum FIFO_MODES {
    /** FIFO mode selection
     *  (000: Bypass mode: FIFO disabled;
     *  001: FIFO mode: stops collecting data when FIFO is full;
     *  010: Reserved;
     *  011: Continuous-to-FIFO mode: Continuous mode until trigger is deasserted, then
     *      FIFO mode;
     *  100: Bypass-to-Continuous mode: Bypass mode until trigger is deasserted, then
     *      Continuous mode;
     *  101: Reserved;
     *  110: Continuous mode: if the FIFO is full, the new sample overwrites the older one;
     *  111: Bypass-to-FIFO mode: Bypass mode until trigger is deasserted, then FIFO
     *      mode.)
     * */
    BYPASS_MODE = 0b000,
    FIFO_MODE = 0b001,
    CONTINUOUS_TO_FIFO_MODE = 0b011,
    BYPASS_TO_CONTINUOUS_MODE = 0b100,
    CONTINUOUS_MODE = 0b110,
    BYPASS_TO_FIFO_MODE = 0b111
};

// The LSM6DSO32 can be used as an I2C or SPI device, use the appropriate constructor for the desired protocol.
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
    /* Functions */
    uint8_t enable_embedded_functions(bool enable);
    uint8_t enable_sensor_hub(bool enable);
    uint8_t enable_sdo_pullup(bool enable);
    uint8_t set_fifo_watermark(short watermark);
    short get_fifo_watermark();
    uint8_t stop_on_WTM(bool enable);
    uint8_t enable_fifo_compression_runtime(bool enable);
    uint8_t enable_fifo_compression(bool enable);
    // TODO: FIFO_CTRL2 ODRCHG_EN ??
    // TODO: UNCOPTR_RATE ?? (FIFO_CTRL2)
    uint8_t set_batching_data_rates(BATCHING_DATA_RATES accel_BDR, BATCHING_DATA_RATES gyro_BDR);
    uint8_t set_timestamp_batching_decimation(TIMESTAMP_BATCHING_DECIMATION decimation);
    uint8_t set_temperature_batching_data_rate(TEMPERATURE_BATCHING_RATE rate);
    uint8_t set_fifo_mode(FIFO_MODES mode);






};



#endif //ARDUINO_LSM6DS032_LSMDS032_H
