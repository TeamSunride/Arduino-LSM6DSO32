//
// Created by robosam2003 on 23/06/2022.
// TeamSunride
#ifndef ARDUINO_LSM6DSO32_LSM6DSO32_H
#define ARDUINO_LSM6DSO32_LSM6DSO32_H

#include "protocol.h"
#include "LSM6DSO32_registers.h"
#include "LSM6DSO32_constants.h"
#include "Vector.h"
#include "dynamicFifo.h" // using dynamically allocated fifo because fifo size is not known at compile time.



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
class LSM6DSO32{ // This could maybe be a child of the IMU class??
protected:
    protocol* device;
    double accel_conversion_factor;
    double gyro_conversion_factor;

    OFFSET_WEIGHT XL_OFFSET_WEIGHT;

    /* 3. Hybrid, based on combined usage of the TAG_CNT field and decimated timestamp sensor */
    uint64_t timestamp_lsb; // stores the timestamp based on updates from the FIFO and the fifo counter
    byte prev_tag_cnt; // stores the previous value of the tag counter from the fifo
    BATCHING_DATA_RATES XL_BDR; // needed for timestamp-timestamp counter hybrid.
    BATCHING_DATA_RATES GY_BDR;

    // For compression algorithm.
    Vector<double, 4> mostRecentAcc;
    Vector<double, 4> mostRecentGyro;
public:
    /**
     * @brief Constructor overload for I2C protocol
     * @param pipe
     * @param freq
     */
    LSM6DSO32(TwoWire *pipe, uint32_t freq);

    /**
     * @brief Constructor overload for SPI protocol
     * @param chipSelect
     * @param spi
     * @param settings
     */
    LSM6DSO32(byte chipSelect, SPIClass& spi, uint freq);

    /**
     * @brief begin the device
     */
    void begin() {
        device->protocol_begin();
        // any other set up etc
    }

    // breakout reg functions to LSM class
    byte read_reg(LSM6DSO32_REGISTER regAddress);
    void read_regs(LSM6DSO32_REGISTER regAddress, byte* outputPointer,  uint length);
    uint8_t write_reg(LSM6DSO32_REGISTER regAddress, byte data);
    uint8_t write_regs(LSM6DSO32_REGISTER regAddress ,byte* writeBuffer, uint length);

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

    /**
     * @brief Set the watermark threshold for the FIFO (max 511)
     * @param watermark
     * @return Status code (0 for success)
     */
    uint8_t set_fifo_watermark(short watermark);

    /**
     * @brief get the watermark threshold for the FIFO
     * @return the watermark threshold for the FIFO (short)
     */
    short get_fifo_watermark();

    /**
     * @brief Enable: FIFO depth is limited to threshold level. (Default: False)
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t stop_on_WTM(bool enable);


    /*
     * Accelerometer and gyroscope batch data rate (BDR) can be configured independently, but the compression
       algorithm is not supported in following configurations:
       1. Both accelerometer and gyroscope are batched in FIFO and max(ODR_XL, ODR_G) ≥ 1.66 kHz;
       2. Accelerometer only or gyroscope only is batched in FIFO and max(ODR_XL, ODR_G) ≥ 3.33 kHz.
     */
    /**
     * @brief Enable/disable the fifo compression during device operation (runtime)
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t enable_fifo_compression_runtime(bool enable);

    /**
     * @brief Enable fifo compression during setup
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t enable_fifo_compression(bool enable);

    /**
     * @brief Set the uncompressed data fifo batching rate
     * @param rate
     * @return Status code (0 for success)
     */
    uint8_t set_uncompressed_data_rate(UNCOMPRESSED_DATA_BATCHING_RATES rate);

    /**
     * @brief Set the fifo batching data rate for accel and gyro
     * @param accel_BDR
     * @param gyro_BDR
     * @return Status code (0 for success)
     */
    uint8_t set_batching_data_rates(BATCHING_DATA_RATES accel_BDR, BATCHING_DATA_RATES gyro_BDR);

    /**
     * @brief Set the timestamp fifo batching decimation. (Timestamp will batch every *decimation* MAX(XL_BDR, GY_BDR) )
     * @param decimation
     * @return Status code (0 for success)
     */
    uint8_t set_timestamp_batching_decimation(TIMESTAMP_BATCHING_DECIMATION decimation);

    /**
     * @brief Set the temperature data fifo batching rate.
     * @param rate
     * @return Status code (0 for success)
     */
    uint8_t set_temperature_batching_data_rate(TEMPERATURE_BATCHING_RATE rate);

    /**
     * @brief Select the fifo mode : (Continuous recommended)
     *  FIFO mode selection
     *      (000: Bypass mode: FIFO disabled;
     *      001: FIFO mode: stops collecting data when FIFO is full;
     *      010: Reserved;
     *      011: Continuous-to-FIFO mode: Continuous mode until trigger is deasserted, then
     *          FIFO mode;
     *      100: Bypass-to-Continuous mode: Bypass mode until trigger is deasserted, then
     *          Continuous mode;
     *      101: Reserved;
     *      110: Continuous mode: if the FIFO is full, the new sample overwrites the older one;
     *      111: Bypass-to-FIFO mode: Bypass mode until trigger is deasserted, then FIFO
     *          mode.)
     *
     * @param mode
     * @return Status code (0 for success)
     */
    uint8_t set_fifo_mode(FIFO_MODES mode);

    /**
     * @brief Enables pulsed data-ready mode
        (0: Data-ready latched mode (returns to 0 only after an interface reading) (default);
        1: Data-ready pulsed mode (the data ready pulses are 75 μs long)
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t set_dataready_pulsed(bool enable);

    /**
     * @brief reset the bdr counter
     * @return Status code (0 for success)
     */
    uint8_t reset_counter_bdr();

    /**
     * @brief True: set Gyro as batch count trigger  |   False: Set Accel as batch count trigger
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t set_gyro_as_batch_count_trigger(bool enable); /// will select accel if enable is false

    /**
     * @brief Set the BDR counter threshold. When this threshold is reached, counter_bdr_flag in LSM_FIFO_STATUS is set high.\n Range: 0-2047 (11 bits)
     * @param threshold
     * @return Status code (0 for success)
     */
    uint8_t set_BDR_counter_threshold(short threshold);

    /**
     * @brief Get the BDR counter threshold
     * @return The BDR counter threshold (short)
     */
    short get_BDR_counter_threshold();

    /**
     * @brief Set the interrupt on INT1 pin.
     * @param interrupt
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t set_INT1_INTERRUPT(INTERRUPTS interrupt, bool enable);

    /**
     * @brief Set the interrupt on INT2 pin.
     * @param interrupt
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t set_INT2_INTERRUPT(INTERRUPTS interrupt, bool enable);

    /**
     * @brief Access the device ID to confirm that we are actually talking with an LSM
     * @return The device ID (should be 0x6C)
     */
    byte who_am_i();

    /**
     * @brief Set the ODR (output data rate) for the accelerometer
     * @param rate
     * @return Status code (0 for success)
     */
    uint8_t set_accel_ODR(OUTPUT_DATA_RATES rate);

    /**
     * @brief Set the accelerometer scale (e.g. +-32G
     * @param scale
     * @return Status code (0 for success)
     */
    uint8_t set_accel_full_scale(ACCEL_FULL_SCALE scale);

    /**
     * @brief Set the ODR (output data rate) for the gyroscope
     * @param rate
     * @return Status code (0 for success)
     */
    uint8_t set_gyro_ODR(OUTPUT_DATA_RATES rate);

    /**
     * Set the
     * @param scale
     * @return
     */
    uint8_t set_gyro_full_scale(GYRO_FULL_SCALE scale);

    /**
     * @brief Enable LPF2 filter for the accelerometer
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t enable_XL_LPF2(bool enable);

    /**
     * @brief Set the interrupts to active low (default false)
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t set_interrupts_active_low(bool enable);

    /**
     * @brief Put SPI into 3-wire mode
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t set_SPI_as_3_wire(bool enable);

    /**
     * @brief Register address automatically incremented during a multiple byte access with a
        serial interface (I²C or SPI). Default value: 1
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_auto_address_increment(bool enable);

    /**
     * @brief Completely reset the device
     * @return Status code (0 for success)
     */
    uint8_t software_reset();

    /**
     * @brief Enable gyroscope sleep mode
     * @param enable
     * @return Status code (0 for success)
     */
    uint8_t set_gyroscope_sleep(bool enable);

    /**
     * @brief If enabled, the accelerometer and gyroscope data-ready signals
                are masked until the settling of the sensor filters is completed.
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_data_ready_mask(bool enable);

    /**
     * @brief Enable the I2C interface (default: I2C enabled)
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_i2c_interface(bool enable);

    /**
     * @brief Enable gyro LPF1 filter. Bandwidth can be selected through gyro_LPF1_bandwidth(byte bandwidth)
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_gyro_LPF1(bool enable);

    /**
     * @brief Put the accelerometer into ultra low power mode
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_accel_ultra_low_power(bool enable);

    /**
     * @brief Enable the rounding function.
     * The rounding function can be used to auto address the device registers for a circular burst-mode read. Basically,\n
        with a multiple read operation the address of the register that is being read goes automatically from the first \n
        register to the last register of the pattern and then goes back to the first one.
     * @param accelEnable
     * @param gyroEnable
     * @return Status Code (0 for success)
     */
    uint8_t enable_rounding(bool accelEnable, bool gyroEnable);

    /**
     * @brief Performs a self test on the accelerometer
     * @return Bool indicating pass(true) or fail(false)
     */
    bool accel_self_test();

    /**
     * @brief Performs a self test on the gyroscope
     * @return Bool indicating pass(true) or fail(false)
     */
    bool gyro_self_test();

    /**
     * @brief Put the accelerometer into high performance mode
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_accel_high_performance_mode(bool enable);

    /**
     * @brief Set the weight of the offset registers. 2^-10 g/LSB or 2^-6 g/LSB
     * @param weight
     * @return Status Code (0 for success)
     */
    uint8_t select_XL_offset_weight(OFFSET_WEIGHT weight);

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

    /**
     * @brief Enable the gyro high performance mode
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_gyro_high_performance_mode(bool enable);

    /**
     * @brief Enable the gyroscope high pass filter. Only enabled if the gyroscope is in high performance mode.
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_gyro_high_pass_filter(bool enable);

    /**
     * @brief Set the gyro high pass filter cutoff.
     * @param cutoff
     * @return Status Code (0 for success)
     */
    uint8_t set_gyro_high_pass_filter_cutoff(GYRO_HIGH_PASS_FILTER_CUTOFF cutoff);

    /**
     * @brief Enable the accelerometer offset block
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_accel_offset_block(bool enable);

    /**
     * @brief Set the filter cutoff for either the accel LPF2 or the accel high pass (selected with accel_high_pass_selection)
     * @param cutoff
     * @return Status Code (0 for success)
     */
    uint8_t set_accel_high_pass_or_LPF2_filter_cutoff(ACCEL_HP_OR_LPF2_CUTOFF cutoff);

    /**
     * @brief Enable accel high pass filter reference mode.
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_accel_high_pass_filter_reference_mode(bool enable);

    /**
     * @brief Enable accel fast settling mode
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t enable_accel_fast_settling_mode(bool enable);

    /**
     * @brief Select high pass filter path for the accel. (false selects the LPF2 path)
     * @param select
     * @return Status Code (0 for success)
     */
    uint8_t accel_high_pass_selection(bool select);

    // TODO: DATA ENABLE (DEN) - CTRL9_XL - see Application note 4.8 - Also CTRL6_C - Unsure whether it's worth implementing.

    /**
     * @brief Enable the counter for the timestamp. - Very important if you are planning to use the Fifo.
     * @param enable
     * @return Status Code (0 for success)
     */
    uint8_t timestamp_counter_enable(bool enable);

    /**
     * @brief Get the temperature data ready signal status
     * @return Status Code (0 for success)
     */
    bool get_temp_drdy_status();

    /**
     * @brief Get the gyro data ready signal status
     * @return Status Code (0 for success)
     */
    bool get_gyr_drdy_status();

    /**
     * @brief Get the accel data ready signal status
     * @return Status Code (0 for success)
     */
    bool get_accel_drdy_status();

    /**
     * @brief Get the temperature from the temperature sensor.
     * @return The temperature (short)
     */
    short get_temperature();

    /**
     * @brief Get the gyro values direct from the registers.
     * @return Status Code (0 for success)
     */
    Vector<double, 3> get_gyro();

    /**
     * @brief Get the accel values direct from the registers
     * @return Status Code (0 for success)
     */
    Vector<double, 3> get_accel();

    /**
     * @brief Get the timestamp value direct from the registers. - 25us per LSB
     * @return
     */
    uint32_t get_timestamp();

    /// A bunch of "TAP" and "WAKE_UP" registers - not being implemented as they are not useful in a rocket context
    // TODO: Look at launch detect interrupt

    /**
     * @brief Set the built in accel offsets
     * @param offsets
     * @return Status Code (0 for success)
     */
    uint8_t set_XL_offset_compensation(Vector<double, 3> offsets);

    /**
     * @brief Get the LSM fifo status. (Fifo flags and num unread)
     * @return Status Code (0 for success)
     */
    LSM_FIFO_STATUS get_fifo_status();

    /**
     * @brief Get the timestamps increment based on the BDR (used internally)
     * @return Status Code (0 for success)
     */
    int get_timestamp_increment();

    /**
     * @brief Pop values from the LSM FIFO, process them (decompression, etc), then pop them onto the accFifo and gyrFifo
     * @param acc_fifo
     * @param gyr_fifo
     * @return Status Code (0 for success)
     */
    uint8_t fifo_pop(Fifo<Vector<double, 4>>& acc_fifo, Fifo<Vector<double, 4>>& gyr_fifo);

    /**
     * @brief Clear the LSM FIFO
     */
    void fifo_clear();

    /**
     * @brief The default configuration for the LSM.
     * @return Status Code (0 for success)
     */
    uint8_t default_configuration();


};



#endif //ARDUINO_LSM6DSO32_LSM6DSO32_H
