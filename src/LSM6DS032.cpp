//
// Created by robos on 23/06/2022.
//
#include "LSM6DS032.h"
#include "Vector.h"

/// Constructors
LSM6DS032::LSM6DS032(TwoWire *pipe, uint32_t freq) { // constructor for I2C protocol
    device = new I2CProtocol(LSM6DS032_DEFAULT_I2C_ADDRESS, pipe, freq);
    accel_conversion_factor = 0.0098*0.978; /// Defaults to +- 32g sensitivity
    gyro_conversion_factor = 0.07; /// defaults to +- 2000dps
    enable_sdo_pullup(true); // pullup for i2c SDA/SDO line - probably best to use external ones
}

LSM6DS032::LSM6DS032(byte chipSelect, SPIClass& spi, SPISettings settings) { // constructor for SPI protocol
    // TODO: is there a way to assert/ensure that the SPI settings are that of the LSM6DS032? - namely, MSB first, SPI mode 2.
    device = new SPIProtocol(chipSelect, spi, settings, READ_BYTE, WRITE_BYTE);
}

// Cheers GitHub copilot ;)

byte LSM6DS032::read_reg(LSM6DS032_REGISTER regAddress) {
    return device->read_reg(regAddress);
}

void LSM6DS032::read_regs(LSM6DS032_REGISTER regAddress, byte *outputPointer, uint length) {
    return device->read_regs(regAddress, outputPointer, length);
}

uint8_t LSM6DS032::write_reg(LSM6DS032_REGISTER regAddress, byte data) {
    return device->write_reg(regAddress, data);
}

uint8_t LSM6DS032::write_regs(LSM6DS032_REGISTER regAddress, byte *data, uint length) {
    return device->write_regs(regAddress, data, length);
}



uint8_t LSM6DS032::enable_embedded_functions(bool enable) {
    byte data= device->read_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS);
    data= data& 0b11000000; // last 6 bits ' must be set to '0' for the correct operation of the device. ' according to the datasheet
    setBit(&data, 7, enable);
    return device->write_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS, data);
}

uint8_t LSM6DS032::enable_sensor_hub(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS);
    data = data & 0b11000000; // last 6 bits ' must be set to '0' for the correct operation of the device. ' according to the datasheet
    setBit(&data, 6, enable);
    return device->write_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS, data);
}

uint8_t LSM6DS032::enable_sdo_pullup(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::PIN_CTRL);
    setBit(&data, 7, 0);  data = data & 0b11000000; // essential for operation of the device
    setBit(&data, 6, enable);
    return device->write_reg(LSM6DS032_REGISTER::PIN_CTRL, data);
}

uint8_t LSM6DS032::set_fifo_watermark(short waterMarkBytes) { /// max watermark 511 (9 bits)
    /* FIFO watermark threshold
        1 LSB = 1 sensor (6 bytes) + TAG (1 byte) written in FIFO
        Watermark flag rises when the number of bytes written in the FIFO is greater than or
        equal to the threshold level.
     */
    // TODO: range checks on watermarkBytes
    uint8_t a = device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL1, (byte)(waterMarkBytes&0xFF)); // WTM0-7
    uint8_t b = device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL2, (byte)((waterMarkBytes>>8)&0x01)); // first bit of FIFO_CTRL2 is WTM8
    return (a | b); // if either writes fail, return true (1 means failure, 0 means success).
}

short LSM6DS032::get_fifo_watermark() {
    return (short)(device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL1) | (((device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL2))&0x01)<<8));
}

uint8_t LSM6DS032::stop_on_WTM(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL2);
    data = data & 0b11010111; // bits 3 and 5 must be zero.
    setBit(&data, 7, enable);
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL2, data);
}

uint8_t LSM6DS032::enable_fifo_compression_runtime(bool enable) { // for enabling/disabling fifo compression at runtime
    // warning: does not check if FIFO_COMP_EN is set in EMB_FUNC_EN_B register.
    //          if FIFO_COMP_EN is not set, then the FIFO compression will not be enabled.
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL2);
    data = data & 0b11010111; // bits 3 and 5 must be zero.
    setBit(&data, 6, enable);
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL2, data);
}

uint8_t LSM6DS032::enable_fifo_compression(bool enable) {
    // TODO: enable_fifo_compression - requires embedded functions access.
}


uint8_t LSM6DS032::set_uncompressed_data_rate(UNCOMPRESSED_DATA_BATCHING_RATES rate) {
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL2);
    data = data & 0b11010001;
    data = (rate<<1) | data;
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL2, data);
}

// The batching data rate is the writing frequency to the fifo
uint8_t LSM6DS032::set_batching_data_rates(BATCHING_DATA_RATES accel_BDR, BATCHING_DATA_RATES gyro_BDR) { // accel and gyro not batched in fifo by default
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL3, ((gyro_BDR<<4) | accel_BDR));
}

uint8_t LSM6DS032::set_timestamp_batching_decimation(TIMESTAMP_BATCHING_DECIMATION decimation) {
    /** Selects decimation for timestamp batching in FIFO. Writing rate will be the maximum
        rate between XL and GYRO BDR divided by decimation decoder.*/
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL4);
    data = data & 0b00110111; // bit 3 must be 0, clear top two bits ready for decimation write
    data = (decimation<<6) | data;
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL4, data);
}

uint8_t LSM6DS032::set_temperature_batching_data_rate(TEMPERATURE_BATCHING_RATE rate) {
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL4);
    data = data & 0b11000111; // bit 3 must be 0, clear bits 4 and 5 ready for temperature rate write
    data = (rate<<4) | data;
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL4, data);
}

uint8_t LSM6DS032::set_fifo_mode(FIFO_MODES mode) { //
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL4);
    data = data & 0b11110000; // bit 3 must be zero, clear bottom 3 bits
    data = mode | data;
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL4, data);
}

uint8_t LSM6DS032::reset_counter_bdr() {
    byte data = device->read_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1);
    setBit(&data, 6, 1);
    return device->write_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1, data);
}

uint8_t LSM6DS032::set_gyro_as_batch_count_trigger(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1);
    setBit(&data, 5, enable);
    return device->write_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1, data);
}

uint8_t LSM6DS032::set_BDR_counter_threshold(short threshold) {
    byte highByte = device->read_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1);
    highByte = highByte & 0b11100000; // 3 and 4 must be 0, clear lower 3 bits
    highByte &= (byte)(threshold>>8);
    return (device->write_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1, highByte) | device->write_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG2, (byte)(threshold&0xFF)));
}

short LSM6DS032::get_BDR_counter_threshold() {
    return (short)(((device->read_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1)&0b00000111))<<8 | device->read_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG2) );
}

uint8_t LSM6DS032::set_INT1_INTERRUPT(INTERRUPTS interrupt, bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::INT1_CTRL);
    setBit(&data, interrupt, enable);
    return device->write_reg(LSM6DS032_REGISTER::INT1_CTRL, data);
}

uint8_t LSM6DS032::set_INT2_INTERRUPT(INTERRUPTS interrupt, bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::INT2_CTRL);
    setBit(&data, interrupt, enable);
    return device->write_reg(LSM6DS032_REGISTER::INT2_CTRL, data);
}

uint8_t LSM6DS032::set_dataready_pulsed(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1);
    data = data & 0b01100111; // bits 3 and 4 must be zero, clear bit 7
    setBit(&data, 7, enable);
    return device->write_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1, data);
}



byte LSM6DS032::who_am_i() {
    return device->read_reg(LSM6DS032_REGISTER::WHO_AM_I);
}

uint8_t LSM6DS032::set_accel_ODR(OUTPUT_DATA_RATES rate) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL1_XL);
    data = data & 0b00001110; // bit 0 must be 0
    data = (rate<<4) | data;
    return device->write_reg(LSM6DS032_REGISTER::CTRL1_XL, data);
}

uint8_t LSM6DS032::set_accel_full_scale(ACCEL_FULL_SCALE scale) {

    switch (scale) {
        case (ACCEL_FULL_SCALE::ACCEL_SCALE_4G): {
            accel_conversion_factor = 0.0098*0.122;
            break;
        }
        case (ACCEL_FULL_SCALE::ACCEL_SCALE_8G): {
            accel_conversion_factor = 0.0098*0.244;
            break;
        }
        case (ACCEL_FULL_SCALE::ACCEL_SCALE_16G): {
            accel_conversion_factor = 0.0098*0.488;
            break;
        }
        case (ACCEL_FULL_SCALE::ACCEL_SCALE_32G): {
            accel_conversion_factor = 0.0098*0.976;
            break;
        }
        default: {
            accel_conversion_factor = 1;
            break;
        }
    }
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL1_XL);
    data = data & 0b11110010;
    data = (scale<<2) | data;
    return device->write_reg(LSM6DS032_REGISTER::CTRL1_XL, data);
}

uint8_t LSM6DS032::set_gyro_ODR(OUTPUT_DATA_RATES rate) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL2_G);
    data = data & 0b00001100; // bit 0 must be 0
    data = (rate<<4) | data;
    return device->write_reg(LSM6DS032_REGISTER::CTRL2_G, data);
}

uint8_t LSM6DS032::set_gyro_full_scale(GYRO_FULL_SCALE scale) {
    switch (scale) { // The 125dps option is not available because that is selected via a different register
        case (GYRO_FULL_SCALE::GYRO_SCALE_250DPS) : {
            gyro_conversion_factor=0.001*8.75;
            break;
        }
        case (GYRO_FULL_SCALE::GYRO_SCALE_500DPS) : {
            gyro_conversion_factor=0.001*17.5;
            break;
        }
        case (GYRO_FULL_SCALE::GYRO_SCALE_1000DPS) : {
            gyro_conversion_factor=0.001*35;
            break;
        }
        case (GYRO_FULL_SCALE::GYRO_SCALE_2000DPS) : {
            gyro_conversion_factor=0.001*70;
            break;
        }
        default: {
            gyro_conversion_factor = 1;
            break;
        }

    }

    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL2_G);
    data = data & 0b11110000; // bit 1 should be zero also, because we dont just want 125 dps.
    data = (scale<<2) | data;
    return device->write_reg(LSM6DS032_REGISTER::CTRL2_G, data);
}

uint8_t LSM6DS032::enable_LPF2(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL1_XL);
    setBit(&data, 1, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL1_XL, data);
}

uint8_t LSM6DS032::set_interrupts_active_low(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL3_C);
    setBit(&data, 5, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL3_C, data);
}

uint8_t LSM6DS032::set_SPI_as_3_wire(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL3_C);
    setBit(&data, 3, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL3_C, data);
}

uint8_t LSM6DS032::enable_auto_address_increment(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL3_C);
    setBit(&data, 1, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL3_C, data);
}

uint8_t LSM6DS032::software_reset() {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL3_C);
    setBit(&data, 0, 1);
    return device->write_reg(LSM6DS032_REGISTER::CTRL3_C, data);
}

uint8_t LSM6DS032::set_gyroscope_sleep(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL4_C);
    setBit(&data, 6, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL4_C, data);
}

uint8_t LSM6DS032::enable_data_ready_mask(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL4_C);
    setBit(&data, 3, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL4_C, data);
}

uint8_t LSM6DS032::enable_i2c_interface(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL4_C);
    setBit(&data, 2, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL4_C, data);
}

uint8_t LSM6DS032::enable_gyro_LPF1(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL4_C);
    setBit(&data, 1, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL4_C, data);
}

uint8_t LSM6DS032::enable_accel_ultra_low_power(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL5_C);
    setBit(&data, 7, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL5_C, data);
}

uint8_t LSM6DS032::enable_rounding(bool accelEnable, bool gyroEnable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL5_C);
    setBit(&data, 5, accelEnable);
    setBit(&data, 6, gyroEnable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL5_C, data);
}

uint8_t LSM6DS032::enable_accel_high_performance_mode(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL6_C);
    setBit(&data, 4, !enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL6_C, data);
}

uint8_t LSM6DS032::gyro_low_pass_filter_bandwidth(byte bandwidth) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL6_C);
    data = data & 0b11111000;
    data = (bandwidth) | data;
    return device->write_reg(LSM6DS032_REGISTER::CTRL6_C, data);
}

uint8_t LSM6DS032::enable_gyro_high_performance_mode(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL7_G);
    setBit(&data, 7, !enable); // enabled is 0.
    return device->write_reg(LSM6DS032_REGISTER::CTRL7_G, data);
}

uint8_t LSM6DS032::enable_gyro_high_pass_filter(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL7_G);
    setBit(&data, 6, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL7_G, data);
}

uint8_t LSM6DS032::set_gyro_high_pass_filter_cutoff(GYRO_HIGH_PASS_FILTER_CUTOFF cutoff) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL7_G);
    data = data & 0b11000010;
    data = (cutoff<< 4) | data;
    return device->write_reg(LSM6DS032_REGISTER::CTRL7_G, data);
}

uint8_t LSM6DS032::enable_accel_offset_block(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL7_G);
    setBit(&data, 1, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL7_G, data);
}

uint8_t LSM6DS032::set_accel_high_pass_or_LPF2_filter_cutoff(ACCEL_HP_OR_LPF2_CUTOFF cutoff) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL8_XL);
    data = data & 0b00011101;
    data = (cutoff<<5) | data;
    return device->write_reg(LSM6DS032_REGISTER::CTRL8_XL, data);
}

uint8_t LSM6DS032::enable_accel_high_pass_filter_reference_mode(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL8_XL);
    setBit(&data, 4, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL8_XL, data);
}

uint8_t LSM6DS032::enable_accel_fast_settling_mode(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL8_XL);
    setBit(&data, 3, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL8_XL, data);
}

uint8_t LSM6DS032::accel_high_pass_selection(bool select) {
    /**
     *  0 for low-pass (LPF2), 1 for high pass
     */
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL8_XL);
    setBit(&data, 2, select);
    return device->write_reg(LSM6DS032_REGISTER::CTRL8_XL, data);
}

uint8_t LSM6DS032::timestamp_counter_enable(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::CTRL10_C);
    setBit(&data, 5, enable);
    return device->write_reg(LSM6DS032_REGISTER::CTRL10_C, data);
}

short LSM6DS032::get_temperature() {
    byte data[2] = {};
    device->read_regs(LSM6DS032_REGISTER::OUT_TEMP_L, data, 2);
    return (short)((data[1]<<8) | data[0]);
}


Vector<double> LSM6DS032::get_gyro() {
    Vector<double> returnVect = {0, 0, 0};
    byte a[6] = {0};
    device->read_regs(LSM6DS032_REGISTER::OUTX_L_G, a, 6);
    returnVect[0] = ((short)((a[1]<<8) | a[0])) * gyro_conversion_factor;
    returnVect[1] = ((short)((a[3]<<8) | a[2])) * gyro_conversion_factor;
    returnVect[2] = ((short)((a[5]<<8) | a[4])) * gyro_conversion_factor;
    return returnVect;
}


Vector<double> LSM6DS032::get_accel() {
    Vector<double> returnVect = {0, 0, 0};
    byte a[6] = {0};
    device->read_regs(LSM6DS032_REGISTER::OUTX_L_A, a, 6);
    returnVect[0] = ((short)((a[1]<<8) | a[0])) * accel_conversion_factor;
    returnVect[1] = ((short)((a[3]<<8) | a[2])) * accel_conversion_factor;
    returnVect[2] = ((short)((a[5]<<8) | a[4])) * accel_conversion_factor;
    return returnVect;
}

uint32_t LSM6DS032::get_timestamp() {
    byte data[4] = {};
    device->read_regs(LSM6DS032_REGISTER::TIMESTAMP0, data, 4);
    return (uint32_t)((data[3]<<24) | (data[2]<<16) | (data[1]<<8) | data[0]);
}


uint8_t LSM6DS032::default_configuration() {
    software_reset();
    set_fifo_watermark(256);
    stop_on_WTM(false);
    /// FIFO compression

    /// BATCHING DATA RATES
    /// FIFO MODE

    //set_dataready_pulsed(true);
    set_gyro_as_batch_count_trigger(false); // using accel instead
    /// BDR threshold
    /// INT1, INT2


    set_accel_ODR(OUTPUT_DATA_RATES::ODR_6667_HZ);
    set_accel_full_scale(ACCEL_FULL_SCALE::ACCEL_SCALE_32G);
    set_gyro_ODR(OUTPUT_DATA_RATES::ODR_6667_HZ);
    set_gyro_full_scale(GYRO_FULL_SCALE::GYRO_SCALE_2000DPS);
    set_interrupts_active_low(false);
    set_SPI_as_3_wire(false);

    enable_auto_address_increment(true);
    enable_data_ready_mask(true);
    enable_i2c_interface(true);
    enable_gyro_LPF1(true);
    enable_rounding(false, false); // rounding??
    enable_accel_high_performance_mode(true);
    gyro_low_pass_filter_bandwidth(0b000);
    enable_gyro_high_performance_mode(true);
    enable_gyro_high_pass_filter(false);

    accel_high_pass_selection(true);
    set_accel_high_pass_or_LPF2_filter_cutoff(ACCEL_HP_OR_LPF2_CUTOFF::ODR_OVER_4);





    timestamp_counter_enable(true);



    return 0;
}

