//
// Created by robos on 24/06/2022.
//

#ifndef ARDUINO_LSM6DS032_LSM6DS032_CONSTANTS_H
#define ARDUINO_LSM6DS032_LSM6DS032_CONSTANTS_H


/* Datasheet: 5.1.2, pg. 37
 bit 0: RW bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the
 device is read. In latter case, the chip will drive SDO at the start of bit 8.
*/
#define WRITE_BYTE 0b00000000
#define READ_BYTE 0b10000000

#define LSM6DS032_DEFAULT_I2C_ADDRESS 0x6A // from back of adafruit breakout board



enum LSM6DS032_OPERATING_MODES {
    /**
     * The LSM6DSO32 has three operating modes available:
     * - only accelerometer active and gyroscope in power-down
     * - only gyroscope active and accelerometer in power-down
     * - both accelerometer and gyroscope sensors active with independent ODR
     */
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

    /**     In Bypass mode (FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0] = 000), the FIFO is not operational
        and it remains empty. Bypass mode is also used to reset the FIFO when in FIFO mode. Datasheet 6.5.1*/
    BYPASS_MODE = 0b000,

    /**     In FIFO mode (FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0] = 001) data from the output
        channels are stored in the FIFO until it is full.\n
            To reset FIFO content, Bypass mode should be selected by writing FIFO_CTRL4
        (0Ah)(FIFO_MODE_[2:0]) to '000'. After this reset command, it is possible to restart FIFO
        mode by writing FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0]) to '001'.\n
            The FIFO buffer memorizes up to 9 kbytes of data (with compression enabled) but the depth
        of the FIFO can be resized by setting the WTM [8:0] bits in FIFO_CTRL1 (07h) and
        FIFO_CTRL2 (08h). If the STOP_ON_WTM bit in FIFO_CTRL2 (08h) is set to '1', FIFO
        depth is limited up to the WTM [8:0] bits in FIFO_CTRL1 (07h) and FIFO_CTRL2 (08h). Datasheet 6.5.2*/
    FIFO_MODE = 0b001,

    /**     In Continuous-to-FIFO mode (FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0] = 011), FIFO
        behavior changes according to the trigger event detected in one of the following interrupt
        events:
        - Single tap
        - Double tap
        - Wake-up
        - Free-fall
        - D6D
        When the selected trigger bit is equal to '1', FIFO operates in FIFO mode.
        When the selected trigger bit is equal to '0', FIFO operates in Continuous mode.  Datasheet 6.5.4 */
    CONTINUOUS_TO_FIFO_MODE = 0b011,

    /**     In Bypass-to-Continuous mode (FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0] = '100'), data
        measurement storage inside FIFO operates in Continuous mode when selected triggers are
        equal to '1', otherwise FIFO content is reset (Bypass mode).
            FIFO behavior changes according to the trigger event detected in one of the following
        interrupt events:
        - Single tap
        - Double tap
        - Wake-up
        - Free-fall
        - D6D       Datasheet 6.5.5*/
    BYPASS_TO_CONTINUOUS_MODE = 0b100,

    /**     Continuous mode (FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0] = 110) provides a continuous
        FIFO update: as new data arrives, the older data is discarded.\n
            A FIFO threshold flag FIFO_STATUS2 (3Bh)(FIFO_WTM_IA) is asserted when the number
        of unread samples in FIFO is greater than or equal to FIFO_CTRL1 (07h) and FIFO_CTRL2
        (08h)(WTM [8:0]).\n
            It is possible to route the FIFO_WTM_IA flag to the INT1 pin by writing in register
        INT1_CTRL (0Dh)(INT1_FIFO_TH) = '1' or to the INT2 pin by writing in register INT2_CTRL
        (0Eh)(INT2_FIFO_TH) = '1'.\n
            A full-flag interrupt can be enabled, INT1_CTRL (0Dh)(INT1_FIFO_FULL) = '1' or
        INT2_CTRL (0Eh)(INT2_FIFO_FULL) = '1', in order to indicate FIFO saturation and
        eventually read its content all at once.\n
            If an overrun occurs, at least one of the oldest samples in FIFO has been overwritten and
        the FIFO_OVR_IA flag in FIFO_STATUS2 (3Bh) is asserted.\n
            In order to empty the FIFO before it is full, it is also possible to pull from FIFO the number of
        unread samples available inFIFO_STATUS1 (3Ah) and FIFO_STATUS2
        (3Bh)(DIFF_FIFO_[9:0]).    Datasheet 6.5.3*/
    CONTINUOUS_MODE = 0b110,

    /**    In Bypass-to-FIFO mode (FIFO_CTRL4 (0Ah)(FIFO_MODE_[2:0] = '111'), data
        measurement storage inside FIFO operates in FIFO mode when selected triggers are equal
        to '1', otherwise FIFO content is reset (Bypass mode).
            FIFO behavior changes according to the trigger event detected in one of the following
        interrupt events:
        - Single tap
        - Double tap
        - Wake-up
        - Free-fall
        - D6D        Datasheet 6.5.6 */
    BYPASS_TO_FIFO_MODE = 0b111
};

enum INTERRUPTS {

};

#endif //ARDUINO_LSM6DS032_LSM6DS032_CONSTANTS_H
