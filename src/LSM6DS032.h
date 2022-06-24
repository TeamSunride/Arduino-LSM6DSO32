//
// Created by robosam2003 on 23/06/2022.
// TeamSunride
#ifndef ARDUINO_LSM6DS032_LSMDS032_H
#define ARDUINO_LSM6DS032_LSMDS032_H

#include "protocol.h"
#include "LSM6DS032_registers.h"
#include "LSM6DS032_constants.h"

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
*/


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
    /**
     *
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t enable_embedded_functions(bool enable);

    /**
     * Enable the sensor hub
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t enable_sensor_hub(bool enable);

    /**
     * Enable the pull up for SDO pin
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t enable_sdo_pullup(bool enable);
    uint8_t set_fifo_watermark(short watermark);
    short get_fifo_watermark();
    uint8_t stop_on_WTM(bool enable);
    uint8_t enable_fifo_compression_runtime(bool enable);
    uint8_t enable_fifo_compression(bool enable);
    // TODO: FIFO_CTRL2 ODRCHG_EN ??  Note: I will do all these TODOs when I have a better understanding of the way the fifo works
    // TODO: UNCOPTR_RATE ?? (FIFO_CTRL2)
    uint8_t set_batching_data_rates(BATCHING_DATA_RATES accel_BDR, BATCHING_DATA_RATES gyro_BDR);
    uint8_t set_timestamp_batching_decimation(TIMESTAMP_BATCHING_DECIMATION decimation);
    uint8_t set_temperature_batching_data_rate(TEMPERATURE_BATCHING_RATE rate);
    uint8_t set_fifo_mode(FIFO_MODES mode);

    /**
     * Enables pulsed data-ready mode
        (0: Data-ready latched mode (returns to 0 only after an interface reading) (default);
        1: Data-ready pulsed mode (the data ready pulses are 75 μs long)
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t set_dataready_pulsed(bool enable);
    // TODO: RST_COUNTER_BDR (COUNTER_BDR_REG1) ??
    // TODO: TRIG_COUNTER_BDR
    // TODO: CNT_BDR_TH





};



#endif //ARDUINO_LSM6DS032_LSMDS032_H
