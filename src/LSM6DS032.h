//
// Created by robosam2003 on 23/06/2022.
// TeamSunride
#ifndef ARDUINO_LSM6DS032_LSMDS032_H
#define ARDUINO_LSM6DS032_LSMDS032_H

#include "protocol.h"
#include "LSM6DS032_registers.h"
#include "LSM6DS032_constants.h"
#include "Vector.h"

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

    // breakout reg functions to LSM class
    byte read_reg(LSM6DS032_REGISTER regAddress);
    void read_regs(LSM6DS032_REGISTER regAddress, byte* outputPointer,  uint length);
    uint8_t write_reg(LSM6DS032_REGISTER regAddress, byte data);
    uint8_t write_regs(LSM6DS032_REGISTER regAddress ,byte* writeBuffer, uint length);

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

    uint8_t set_uncompressed_data_rate(UNCOMPRESSED_DATA_BATCHING_RATES rate);
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
    uint8_t reset_counter_bdr();
    uint8_t set_gyro_as_batch_count_trigger(bool enable); /// will select accel if enable is false
    uint8_t set_BDR_counter_threshold(short threshold);
    short get_BDR_counter_threshold();
    uint8_t set_INT1_INTERRUPT(INTERRUPTS interrupt, bool enable);
    uint8_t set_INT2_INTERRUPT(INTERRUPTS interrupt, bool enable);
    byte who_am_i();
    uint8_t set_accel_ODR(OUTPUT_DATA_RATES rate);
    uint8_t set_accel_full_scale(ACCEL_FULL_SCALE scale);
    uint8_t set_gyro_ODR(OUTPUT_DATA_RATES rate);
    uint8_t set_gyro_full_scale(GYRO_FULL_SCALE scale);
    uint8_t set_interrupts_active_low(bool enable);
    uint8_t set_SPI_as_3_wire(bool enable);

    /**
     * Register address automatically incremented during a multiple byte access with a
        serial interface (I²C or SPI). Default value: 1
     * @param enable
     * @return
     */
    uint8_t enable_auto_address_increment(bool enable);
    uint8_t software_reset();
    uint8_t set_gyroscope_sleep(bool enable);
    /// INT2_on_INT1 - Necessary?
    uint8_t enable_data_ready_mask(bool enable);
    uint8_t enable_i2c_interface(bool enable);
    uint8_t enable_gyro_LPF1(bool enable);
    uint8_t enable_accel_ultra_low_power(bool enable);
    uint8_t enable_rounding(bool accelEnable, bool gyroEnable);
    /// self test stuff ?? (CTRL5_C)
    /// TRIG_EN, LVL1_EN, LVL2_EN ??
    u_int8_t enable_accel_high_performance_mode(bool enable);
    // TODO: gyroscope low pass filter bandwidth

    uint8_t default_configuration();


    Vector<double> get_accel();
    Vector<double> get_gyro();









};



#endif //ARDUINO_LSM6DS032_LSMDS032_H
