//
// Created by robosam2003 on 23/06/2022.
// TeamSunride
#ifndef ARDUINO_LSM6DS032_LSMDS032_H
#define ARDUINO_LSM6DS032_LSMDS032_H

#include "protocol.h"
#include "LSM6DS032_registers.h"
#include "LSM6DS032_constants.h"
#include "Vector.h"
#include "Fifo.h"



#define DEBUG Serial.printf("%s %d\n", __FILE__, __LINE__)
/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
 * Application note: https://www.st.com/resource/en/application_note/dm00517282-lsm6dso-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf
*/


struct LSM_FIFO_STATUS {
    ///  0: FIFO filling is lower than WTM; 1: FIFO filling is equal to or greater than WTM)
    bool watermark_flag;

    /// (0: FIFO is not completely filled; 1: FIFO is completely filled)
    bool overrun_flag;

    /// (0: FIFO is not full; 1: FIFO will be full at the next ODR)
    bool smart_fifo_full_flag;

    /// Counter BDR reaches the CNT_BDR_TH_[10:0] threshold set in
    //  COUNTER_BDR_REG1 (0Bh) and COUNTER_BDR_REG2 (0Ch). Default value: 0
    //  This bit is reset when these registers are read.
    bool counter_bdr_flag;

    /// Latched FIFO overrun status. Default value: 0
    //  This bit is reset when this register is read.
    bool fifo_ovr_latched;

    /// Number of unread sensor data (TAG + 6 bytes) stored in FIFO. Default value: 00
    uint16_t num_fifo_unread;
};

// The LSM6DSO32 can be used as an I2C or SPI device, use the appropriate constructor for the desired protocol.
class LSM6DS032{ // This could maybe be a child of the IMU class??
protected:
    protocol* device;
    double accel_conversion_factor;
    double gyro_conversion_factor;
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

    uint8_t enable_LPF2(bool enable);
    uint8_t set_interrupts_active_low(bool enable);
    uint8_t set_SPI_as_3_wire(bool enable);

    /**
     * Register address automatically incremented during a multiple byte access with a
        serial interface (I²C or SPI). Default value: 1
     * @param enable
     * @return Status Code (0 for success)
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
    /// Weight of user offsets

    /**
     * Gyroscope low pass filter bandwidth - Datasheet 9.17 - pg 72 - Table 60
     * FTYPE [2:0]|   12.5 Hz  |  26 Hz  |   52 Hz   |   104 Hz   |   208 Hz   |   416 Hz   |   833 Hz   |   1.67 kHz   |   3.33 kHz   |   6.67 kHz
     *
        000       |   4.2      |  8.3    |   16.6    |   33.0     |   67.0     |   136.6    |   239.2    |   304.2      |   328.5      |   335.5
        001       |   4.2      |  8.3    |   16.6    |   33.0     |   67.0     |   130.5    |   192.4    |   220.7      |   229.6      |   232.0
        010       |   4.2      |  8.3    |   16.6    |   33.0     |   67.0     |   120.3    |   154.2    |   166.6      |   170.1      |   171.1
        011       |   4.2      |  8.3    |   16.6    |   33.0     |   67.0     |   137.1    |   281.8    |   453.2      |   559.2      |   609.0
        100       |   4.2      |  8.3    |   16.7    |   33.0     |   62.4     |   86.7     |   96.6     |   99.6       |   100.4      |   100.6
        101       |   4.2      |  8.3    |   16.8    |   31.0     |   43.2     |   48.0     |   49.4     |   49.8       |   49.9       |   49.9
        110       |   4.1      |  7.8    |   13.4    |   19.0     |   23.1     |   24.6     |   25.0     |   25.1       |   25.1       |   25.1
        111       |   3.9      |  6.7    |   9.7     |   11.5     |   12.2     |   12.4     |   12.5     |   12.5       |   12.5       |   12.5
     * @param bandwidth
     * @return Status Code (0 for success)
     */
    uint8_t gyro_LPF1_bandwidth(byte bandwidth);
    uint8_t enable_gyro_high_performance_mode(bool enable);
    uint8_t enable_gyro_high_pass_filter(bool enable);
    uint8_t set_gyro_high_pass_filter_cutoff(GYRO_HIGH_PASS_FILTER_CUTOFF cutoff);
    uint8_t enable_accel_offset_block(bool enable);

    uint8_t set_accel_high_pass_or_LPF2_filter_cutoff(ACCEL_HP_OR_LPF2_CUTOFF cutoff);
    uint8_t enable_accel_high_pass_filter_reference_mode(bool enable);
    uint8_t enable_accel_fast_settling_mode(bool enable);
    uint8_t accel_high_pass_selection(bool select);
    /// LOW_PASS_ON_6D: omit?

    // TODO: WHAT IS DATA ENABLE (DEN) ?? - CTRL9_XL
    uint8_t timestamp_counter_enable(bool enable);
    /// ALL_INT_SRC
    /// WAKE_UP_SRC
    /// TAP_SRC
    /// D6D_SRC
    /// STATUS_REG
    short get_temperature();
    Vector<double, 3> get_gyro();
    Vector<double, 3> get_accel();
    uint32_t get_timestamp(); /// Best representation? - resolution is 25us / LSB
    /// A bunch of "TAP" and "WAKE_UP" registers - not being implemented as they are not useful in a rocket context

    // TODO: FIFO stuff
    LSM_FIFO_STATUS get_fifo_status();
    uint8_t fifo_pop(Fifo<Vector<double, 3>>& accFifo, Fifo<Vector<double, 3>>& gyrFifo);






    uint8_t default_configuration();












};



#endif //ARDUINO_LSM6DS032_LSMDS032_H
