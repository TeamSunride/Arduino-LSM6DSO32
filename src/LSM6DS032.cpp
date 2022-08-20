//
// Created by robos on 23/06/2022.
//
#include "LSM6DS032.h"

/// Constructors
LSM6DS032::LSM6DS032(TwoWire *pipe, uint32_t freq) { // constructor for I2C protocol
    device = new I2CProtocol(LSM6DS032_DEFAULT_I2C_ADDRESS, pipe, freq);
    accel_conversion_factor = 0.0098*0.978; /// Defaults to +- 32g sensitivity
    gyro_conversion_factor = 0.07; /// defaults to +- 2000dps
    enable_sdo_pullup(true); // pullup for i2c SDA/SDO line - probably best to use external ones as well though
    timestamp_lsb = 0;
    prev_tag_cnt = 0;
}

LSM6DS032::LSM6DS032(byte chipSelect, SPIClass& spi, SPISettings settings) { // constructor for SPI protocol
    // TODO: is there a way to assert/ensure that the SPI settings are that of the LSM6DS032? - namely, MSB first, SPI mode 2.
    device = new SPIProtocol(chipSelect, spi, settings, READ_BYTE, WRITE_BYTE);
    accel_conversion_factor = 0.0098*0.978; /// Defaults to +- 32g sensitivity
    gyro_conversion_factor = 0.07; /// defaults to +- 2000dps
    timestamp_lsb = 0;
    prev_tag_cnt = 0;
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
    /*
     * Accelerometer and gyroscope batch data rate (BDR) can be configured independently, but the compression
       algorithm is not supported in following configurations:
       1. Both accelerometer and gyroscope are batched in FIFO and max(ODR_XL, ODR_G) ≥ 1.66 kHz;
       2. Accelerometer only or gyroscope only is batched in FIFO and max(ODR_XL, ODR_G) ≥ 3.33 kHz.
     */
    enable_embedded_functions(true);
    byte data = device->read_reg(LSM6DS032_EMBEDDED_REGISTER::EMB_FUNC_EN_B);
    setBit(&data, 3, enable);
    uint8_t stat = device->write_reg(LSM6DS032_EMBEDDED_REGISTER::EMB_FUNC_EN_B, data);
    enable_embedded_functions(false);
    return stat;
}


uint8_t LSM6DS032::set_uncompressed_data_rate(UNCOMPRESSED_DATA_BATCHING_RATES rate) {
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL2);
    data = data & 0b11010001;
    data = (rate<<1) | data;
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL2, data);
}

// The batching data rate is the writing frequency to the fifo
uint8_t LSM6DS032::set_batching_data_rates(BATCHING_DATA_RATES accel_BDR, BATCHING_DATA_RATES gyro_BDR) { // accel and gyro not batched in fifo by default
    XL_BDR = accel_BDR;
    GY_BDR = gyro_BDR;
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
    setBit(&data, 2, enable);
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

uint8_t LSM6DS032::gyro_LPF1_bandwidth(byte bandwidth) {
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


Vector<double, 3> LSM6DS032::get_gyro() {
    Vector<double, 3> returnVect = {0, 0, 0};
    byte a[6] = {0};
    device->read_regs(LSM6DS032_REGISTER::OUTX_L_G, a, 6);
    returnVect[0] = ((short)((a[1]<<8) | a[0])) * gyro_conversion_factor;
    returnVect[1] = ((short)((a[3]<<8) | a[2])) * gyro_conversion_factor;
    returnVect[2] = ((short)((a[5]<<8) | a[4])) * gyro_conversion_factor;
    return returnVect;
}


Vector<double, 3> LSM6DS032::get_accel() {
    Vector<double, 3> returnVect = {0, 0, 0};
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


LSM_FIFO_STATUS LSM6DS032::get_fifo_status() {
    struct LSM_FIFO_STATUS status = {false,false,false,false,false,0};
    byte a[2] = {};
    device->read_regs(LSM6DS032_REGISTER::FIFO_STATUS1, a, 2);
    status.watermark_flag = getBit(a[1], 7);
    status.overrun_flag = getBit(a[1], 6);
    status.smart_fifo_full_flag = getBit(a[1], 5);
    status.counter_bdr_flag = getBit(a[1], 4);
    status.fifo_ovr_latched = getBit(a[1], 3);
    status.num_fifo_unread = ((a[1] & 0b00000011) << 8) | a[0];

    return status;
}

int LSM6DS032::get_timestamp_increment(){
    switch (max(XL_BDR, GY_BDR)) {
        case NO_BATCHING: // should never occur
            break;
        case BDR_XL_1_6Hz:
            // 1.6Hz XL and 6.5Hz GYRO are not being handled here.
            // If you are using such a low data rate do you really need to use the fifo?
            break;
        case BDR_12_5Hz:
            return 3072; // (1/13.0208333...)/25e-6 = 3072
            break;
        case BDR_26Hz:
            return 1536; // (1/26.041666...)/25e-6 = 1536
            break;
        case BDR_52Hz:
            return 768; // (1/52.08333...)/25e-6 = 768
            break;
        case BDR_104Hz:
            return 384; // (1/104.1666...)/25e-6 = 384
            break;
        case BDR_208Hz:
            return 192; // (1/208.3333...)/25e-6 = 192
            break;
        case BDR_417Hz:
            return 96; // (1/416.666...)/25e-6 = 96
            break;
        case BDR_833Hz:
            return 48; // (1/833.333...)/25e-6 = 48
            break;
        case BDR_1667Hz:
            return 24; // (1/1666.666...)/25e-6 = 24
            break;
        case BDR_3333Hz:
            return 12; // (1/3333.33...)/25e-6 = 12
            break;
        case BDR_6667Hz:
            return 6; // (1/6666.66...)/25e-6 = 6
        default:
            break;
    }
    return 0;
}

byte getNumOnes(byte data) {
    byte numOnes = 0;
    for (int i=0;i<8;i++) {
        if (getBit(data, i)) {
            numOnes += 1;
        }

    }
    return numOnes;
}

void LSM6DS032::fifo_clear() {
    LSM_FIFO_STATUS fifo_status = this->get_fifo_status();
    int num_unread = fifo_status.num_fifo_unread;
    byte data[7] = {};
    for (int i=0;i<num_unread;i++) {
        device->read_regs(LSM6DS032_REGISTER::FIFO_DATA_OUT_TAG, data, 7);
    }
}

int8_t int5_t(int fiveBits){
    int8_t res = 0;
    for (int i=0;i<4;i++) {
        (getBit(fiveBits, i)) ? res += pow(2, i) : 0;
    }
    getBit(fiveBits, 4) ? res -= pow(2, 4) : 0; // it's a 5 bit signed int (two's complement) so the last bit (5) represents -2^4
    return res;
}



uint8_t LSM6DS032::fifo_pop(Fifo<Vector<double, 4>> &acc_fifo, Fifo<Vector<double, 4>> &gyr_fifo) { // three data fields then one timestamp.
    /*
     * When FIFO is enabled and the mode is different from Bypass, reading the FIFO output registers return the oldest
       FIFO sample set. Whenever these registers are read, their content is moved to the SPI/I²C/MIPI I3C output
       buffer.
       FIFO slots are ideally shifted up one level in order to release room for a new sample, and the FIFO output
       registers load the current oldest value stored in the FIFO buffer.
       The recommended way to retrieve data from the FIFO is the following:
            1. Read the FIFO_STATUS1 and FIFO_STATUS2 registers to check how many words are stored in the FIFO.
               This information is contained in the DIFF_FIFO_[9:0] bits.
            2. For each word in FIFO, read the FIFO word (tag and output data) and interpret it on the basis of the FIFO
               tag.
            3. Go to step 1          - application note 9.8 (page 104)
     */
    /*
     * As shown in Table 87, using FIFO compression introduces a latency of 2 / BDR, since the compression acts on a
       window of three BDR. - application note  9.10.1
     */
    byte data[7] = {};
    device->read_regs(LSM6DS032_REGISTER::FIFO_DATA_OUT_TAG, data, 7);
    byte tag = data[0] >> 3;

    // The tag counter is synchronized with the highest BDR. It is for making sense of the time slot in which compressed sensor data occurred. - See application note 9.4 (page 94)
    byte tag_cnt = (data[0] & 0b00000110) >> 1;
    byte tag_parity = data[0] & 0b00000001;
    if ((getNumOnes(data[0]) % 2) != 0) {
        Serial.printf("Tag parity error: %d\n", getNumOnes(data[0]));
    }


    if (Serial)Serial.printf("Tag: 0X%X\n", tag);
    //if (Serial)Serial.printf("Tag Counter: %d\n", tag_cnt);
    int i=0;
    while ((((prev_tag_cnt+i)%4) != tag_cnt)) { i++; }

    timestamp_lsb += i*get_timestamp_increment();
    prev_tag_cnt = tag_cnt;



    switch (tag) {
        /**
         *  NC, not compressed, if the difference between the actual and previous data is higher than 128 LSB: one
            sensor sample is stored in one FIFO word; - application note 9.10 (page 107)
         */
        case (FIFO_TAG::GYRO_NC): // 0x01 // Uncompressed data - Data(i)
        {
            // NC, non-compressed, associated to the actual time slot - Application note 9.10.1 (page 108)
            // NC - time slot data(i)
            double x = (short)((data[2]<<8) | data[1]) * gyro_conversion_factor;
            double y = (short)((data[4]<<8) | data[3]) * gyro_conversion_factor;
            double z = (short)((data[6]<<8) | data[5]) * gyro_conversion_factor;
            auto t = static_cast<double>(timestamp_lsb);
            gyr_fifo.push(Vector<double, 4> {x,y,z,t});
            mostRecentGyro = Vector<double, 4> {x,y,z,t}; // update mostRecentGyro to the latest reading that was pushed
            break;
        }

        case(FIFO_TAG::ACCEL_NC): // 0x02 // Uncompressed data - Data(i)
        {
            // NC, non-compressed, associated to the actual time slot - Application note 9.10.1 (page 108)
            // NC - time slot data(i)
            double x = (short)((data[2]<<8) | data[1]) * accel_conversion_factor;
            double y = (short)((data[4]<<8) | data[3]) * accel_conversion_factor;
            double z = (short)((data[6]<<8) | data[5]) * accel_conversion_factor;
            auto t = static_cast<double>(timestamp_lsb);
            acc_fifo.push(Vector<double, 4> {x,y,z,t});
            mostRecentAcc = Vector<double, 4> {x,y,z,t}; // update mostRecentAcc to the latest reading that was pushed
            break;
        }

        case (FIFO_TAG::TEMPERATURE): // 0x03 // Uncompressed
        {
            // First two bytes contain temperature data, other data is zeros - See application note Table 81 (page 95)
            double temp = (short)((data[2]<<8) | data[1]);
            // TODO: implement temperature fifo

            break;
        }

        case (FIFO_TAG::TIMESTAMP): // 0x04 // Uncompressed - Data(i) - updates the timestamp field.
        {
             /* As shown in Table 82, timestamp data contain also some meta-information which can be used to detect a BDR
                change if the CFG-Change sensor is not batched in FIFO - application note 9.5 (page 95)
             */
            // First 4 bytes contain timestamp, rest is metadata about the BDR - See application note Table 82 (page 95)
            timestamp_lsb = (data[4]<<24) | (data[3]<<16) | (data[2]<<8) | data[1]; // update the timestamp_lsb
            byte BDR_SHUB = data[5] & 0b00001111; // Sensor hub BDR
            XL_BDR = (BATCHING_DATA_RATES)(data[6] & 0b00001111); // update Accelerometer BDR
            GY_BDR = (BATCHING_DATA_RATES)((data[6] & 0b11110000) >> 4); // update Gyroscope BDR
            break;
        }

        // cfg change - 0x05 - Not implemented.

        case(FIFO_TAG::ACCEL_NC_T_2): // 0x06
        {
            // NC_T_2, non-compressed, associated to two times the previous time slot; data(i-2) - application note 9.10.1 (page 108)
            double x = (short)((data[2]<<8) | data[1]) * accel_conversion_factor;
            double y = (short)((data[4]<<8) | data[3]) * accel_conversion_factor;
            double z = (short)((data[6]<<8) | data[5]) * accel_conversion_factor;
            auto t = static_cast<double>(timestamp_lsb - get_timestamp_increment()*2); // data(i-2)
            acc_fifo.push(Vector<double, 4>{x,y,z,t});
            mostRecentAcc = Vector<double, 4> {x,y,z,t}; // update mostRecentAcc to the latest reading that was pushed
            break;
        }

        case(FIFO_TAG::ACCEL_NC_T_1): //0x07
        {
            // NC_T_1, non-compressed, associated to the previous time slot; data(i-1) - application note 9.10.1 (page 108)
            double x = (short)((data[2]<<8) | data[1]) * accel_conversion_factor;
            double y = (short)((data[4]<<8) | data[3]) * accel_conversion_factor;
            double z = (short)((data[6]<<8) | data[5]) * accel_conversion_factor;
            auto t = static_cast<double>(timestamp_lsb - get_timestamp_increment()*1); // data(i-1)
            acc_fifo.push(Vector<double, 4> {x,y,z,t});
            mostRecentAcc = Vector<double, 4> {x,y,z,t}; // update mostRecentAcc to the latest reading that was pushed
            break;
        }


        case(FIFO_TAG::ACCEL_2_X_C): // 0x08 // low compression
        {
            /**
             * 2xC, low compression, if the difference between the actual and previous data between 16 and 128 LSB: two
               sensor samples are stored in one FIFO word; application note 9.10 (page 107)
             */
            // 2xC, low compression; diff(i - 2), diff(i - 1) - application note (page 108)
            // data(i) = diff(i) + data(i-1)

            // 8-bit signed data - See application note Table 88 (page 109)
            //Vector<double, 4> v3 = acc_fifo.peekFront(); // data(i-3)

            // data(i - 2)
            double datax2 = ((int8_t)data[1] * accel_conversion_factor) + mostRecentAcc[0]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double datay2 = ((int8_t)data[2] * accel_conversion_factor) + mostRecentAcc[1]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double dataz2 = ((int8_t)data[3] * accel_conversion_factor) + mostRecentAcc[2]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double t2 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*2); // data(i-2)

            // data(i - 1)
            double datax1 = ((int8_t)data[4] * accel_conversion_factor) + datax2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double datay1 = ((int8_t)data[5] * accel_conversion_factor) + datay2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double dataz1 = ((int8_t)data[6] * accel_conversion_factor) + dataz2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double t1 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*1); // data(i-1)

            acc_fifo.push(Vector<double, 4> {datax2, datay2, dataz2, t2});
            acc_fifo.push(Vector<double, 4> {datax1, datay1, dataz1, t1});
            mostRecentAcc = Vector<double, 4> {datax1, datay1, dataz1, t1}; // update mostRecentAcc to the latest reading that was pushed
            break;
        }

        case(FIFO_TAG::ACCEL_3_X_C): // 0x09 // high compression
        {
            /*
             * 3xC, high compression, if the difference between the actual and previous data is less than 16 LSB: three
               sensor samples are stored in one FIFO word. application note 9.10 (page 107)
             */
            // 3xC, high compression; diff(i - 2), diff(i - 1), diff(i) - application note (page 108)
            short FIFO_DATA_OUT_X = (short)((data[2]<<8) | data[1]);
            short FIFO_DATA_OUT_Y = (short)((data[4]<<8) | data[3]);
            short FIFO_DATA_OUT_Z = (short)((data[6]<<8) | data[5]);

            // 5 bit signed data - See application note Table 89 (page 109)
            //Vector<double, 4> v3 = acc_fifo.peekFront(); // data(i-3)
            // data(i - 2)
            double datax2 = (int5_t (FIFO_DATA_OUT_X & 0b0000000000011111)        * accel_conversion_factor) + mostRecentAcc[0]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double datay2 = (int5_t((FIFO_DATA_OUT_X & 0b0000001111100000) >> 5)  * accel_conversion_factor) + mostRecentAcc[1]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double dataz2 = (int5_t((FIFO_DATA_OUT_X & 0b0111110000000000) >> 10) * accel_conversion_factor) + mostRecentAcc[2]; // data(i - 2) = diff(i - 2) + data(i - 3)

            double t2 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*2);

            // data(i - 1)
            double datax1 = (int5_t (FIFO_DATA_OUT_Y & 0b0000000000011111)        * accel_conversion_factor) + datax2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double datay1 = (int5_t((FIFO_DATA_OUT_Y & 0b0000001111100000) >> 5)  * accel_conversion_factor) + datay2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double dataz1 = (int5_t((FIFO_DATA_OUT_Y & 0b0111110000000000) >> 10) * accel_conversion_factor) + dataz2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double t1 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*1);

            // data(i)
            double datax0 = (int5_t (FIFO_DATA_OUT_Z & 0b0000000000011111)        * accel_conversion_factor) + datax1; // data(i) = diff(i) + data(i - 1)
            double datay0 = (int5_t((FIFO_DATA_OUT_Z & 0b0000001111100000) >> 5)  * accel_conversion_factor) + datay1; // data(i) = diff(i) + data(i - 1)
            double dataz0 = (int5_t((FIFO_DATA_OUT_Z & 0b0111110000000000) >> 10) * accel_conversion_factor) + dataz1; // data(i) = diff(i) + data(i - 1)
            double t0 = static_cast<double>(timestamp_lsb);

            acc_fifo.push(Vector<double, 4> {datax2, datay2, dataz2, t2});
            acc_fifo.push(Vector<double, 4> {datax1, datay1, dataz1, t1});
            acc_fifo.push(Vector<double, 4> {datax0, datay0, dataz0, t0});
            mostRecentAcc = Vector<double, 4> {datax0, datay0, dataz0, t0}; // update mostRecentAcc to the latest reading that was pushed

            // well that was _fun_
            break;
        }

        case (FIFO_TAG::GYRO_NC_T_2): // 0x0A //
        {
            // NC_T_2, non-compressed, associated to two times the previous time slot; (i-2) - application note 9.10.1 (page 108)
            double x = (short)((data[2]<<8) | data[1]) * gyro_conversion_factor;
            double y = (short)((data[4]<<8) | data[3]) * gyro_conversion_factor;
            double z = (short)((data[6]<<8) | data[5]) * gyro_conversion_factor;
            double t = static_cast<double>(timestamp_lsb - get_timestamp_increment()*2);
            gyr_fifo.push(Vector<double, 4> {x, y, z, t});
            mostRecentGyro = Vector<double, 4> {x, y, z, t}; // update mostRecentGyro to the latest reading that was pushed
            break;
        }

        case (FIFO_TAG::GYRO_NC_T_1): // 0x0B
        {
            // NC_T_1, non-compressed, associated to the previous time slot; (i-1) - application note 9.10.1 (page 108)
            double x = (short)((data[2]<<8) | data[1]) * gyro_conversion_factor;
            double y = (short)((data[4]<<8) | data[3]) * gyro_conversion_factor;
            double z = (short)((data[6]<<8) | data[5]) * gyro_conversion_factor;
            double t = static_cast<double>(timestamp_lsb - get_timestamp_increment()*1);
            gyr_fifo.push(Vector<double, 4> {x, y, z, t});
            mostRecentGyro = Vector<double, 4> {x, y, z, t}; // update mostRecentGyro to the latest reading that was pushed
            break;
        }

        case (FIFO_TAG::GYRO_2_X_C): // 0X0C // low compression
        {
            /**
             * 2xC, low compression, if the difference between the actual and previous data between 16 and 128 LSB: two
               sensor samples are stored in one FIFO word; application note 9.10 (page 107)
             */
            // 2xC, low compression; diff(i - 2), diff(i - 1) - application note (page 108)
            // data(i) = diff(i) + data(i-1)

            // 8 bit signed data - See application note Table 88 (page 109)
            //Vector<double, 4> v3 = gyr_fifo.peekFront(); // data(i-3)
            double datax2 = (data[1] * gyro_conversion_factor) + mostRecentGyro[0]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double datay2 = (data[2] * gyro_conversion_factor) + mostRecentGyro[1]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double dataz2 = (data[3] * gyro_conversion_factor) + mostRecentGyro[2]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double t2 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*2); // data(i-2)


            double datax1 = (data[4] * gyro_conversion_factor) + datax2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double datay1 = (data[5] * gyro_conversion_factor) + datay2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double dataz1 = (data[6] * gyro_conversion_factor) + dataz2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double t1 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*1); // data(i-1)

            gyr_fifo.push(Vector<double, 4> {datax2, datay2, dataz2, t2});
            gyr_fifo.push(Vector<double, 4> {datax1, datay1, dataz1, t1});
            mostRecentGyro = Vector<double, 4> {datax1, datay1, dataz1, t1}; // update mostRecentGyro to the latest reading that was pushed
            break;
        }

        case (FIFO_TAG::GYRO_3_X_C): // 0x0D // high compression
        {
            /*
             * 3xC, high compression, if the difference between the actual and previous data is less than 16 LSB: three
               sensor samples are stored in one FIFO word. application note 9.10 (page 107)
             */
            // 3xC, high compression; diff(i - 2), diff(i - 1), diff(i) - application note (page 108)
            short FIFO_DATA_OUT_X = (short)((data[2]<<8) | data[1]);
            short FIFO_DATA_OUT_Y = (short)((data[4]<<8) | data[3]);
            short FIFO_DATA_OUT_Z = (short)((data[6]<<8) | data[5]);

            // 5 bit signed data - See application note Table 89 (page 109)

            // data(i - 2)
            double datax2 = (int5_t (FIFO_DATA_OUT_X & 0b0000000000011111)        * gyro_conversion_factor) + mostRecentGyro[0]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double datay2 = (int5_t((FIFO_DATA_OUT_X & 0b0000001111100000) >> 5)  * gyro_conversion_factor) + mostRecentGyro[1]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double dataz2 = (int5_t((FIFO_DATA_OUT_X & 0b0111110000000000) >> 10) * gyro_conversion_factor) + mostRecentGyro[2]; // data(i - 2) = diff(i - 2) + data(i - 3)
            double t2 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*2);

            // data(i - 1)
            double datax1 = (int5_t (FIFO_DATA_OUT_Y & 0b0000000000011111)        * gyro_conversion_factor) + datax2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double datay1 = (int5_t((FIFO_DATA_OUT_Y & 0b0000001111100000) >> 5)  * gyro_conversion_factor) + datay2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double dataz1 = (int5_t((FIFO_DATA_OUT_Y & 0b0111110000000000) >> 10) * gyro_conversion_factor) + dataz2; // data(i - 1) = diff(i - 1) + data(i - 2)
            double t1 = static_cast<double>(timestamp_lsb - get_timestamp_increment()*1);

            // data(i)
            double datax0 = (int5_t (FIFO_DATA_OUT_Z & 0b0000000000011111)        * gyro_conversion_factor) + datax1; // data(i) = diff(i) + data(i - 1)
            double datay0 = (int5_t((FIFO_DATA_OUT_Z & 0b0000001111100000) >> 5)  * gyro_conversion_factor) + datay1; // data(i) = diff(i) + data(i - 1)
            double dataz0 = (int5_t((FIFO_DATA_OUT_Z & 0b0111110000000000) >> 10) * gyro_conversion_factor) + dataz1; // data(i) = diff(i) + data(i - 1)
            double t0 = static_cast<double>(timestamp_lsb);

            gyr_fifo.push(Vector<double, 4> {datax2, datay2, dataz2, t2});
            gyr_fifo.push(Vector<double, 4> {datax1, datay1, dataz1, t1});
            gyr_fifo.push(Vector<double, 4> {datax0, datay0, dataz0, t0});
            mostRecentGyro = Vector<double, 4> {datax0, datay0, dataz0, t0}; // update mostRecentGyro to the latest reading that was pushed

            // well that was _fun_
            break;
        }


        // 0x0E Sensor Hub Slave 0
        // 0x0F Sensor Hub Slave 1
        // 0x10 Sensor Hub Slave 2
        // 0x11 Sensor Hub Slave 3
        // 0x12 Step Counter
        // 0x19 Sensor Hub Nack
        default:
            // no nothing
            break;
    }
    return 0;

}



uint8_t LSM6DS032::default_configuration() {
    software_reset();
    set_fifo_watermark(511);
    stop_on_WTM(false);
    /// FIFO compression
    set_uncompressed_data_rate(UNCOMPRESSED_DATA_BATCHING_RATES::UNCOPTR_8);

    /*e
     * Accelerometer and gyroscope batch data rate (BDR) can be configured independently, but the compression
       algorithm is not supported in following configurations:
       1. Both accelerometer and gyroscope are batched in FIFO and max(ODR_XL, ODR_G) ≥ 1.66 kHz;
       2. Accelerometer only or gyroscope only is batched in FIFO and max(ODR_XL, ODR_G) ≥ 3.33 kHz.
     */
    /// Enable FIFO compression
    enable_fifo_compression(true);
    enable_fifo_compression_runtime(true);

    /// BATCHING DATA RATES
    set_batching_data_rates(BATCHING_DATA_RATES::BDR_208Hz, BATCHING_DATA_RATES::NO_BATCHING); // accel, gyro
    set_timestamp_batching_decimation(TIMESTAMP_BATCHING_DECIMATION::DECIMATION_8); // timestamp decimation

    /// FIFO MODE
    set_fifo_mode(FIFO_MODES::CONTINUOUS_MODE);

    set_dataready_pulsed(true);
    set_gyro_as_batch_count_trigger(false); // using accel instead
    timestamp_counter_enable(true); // VITAL if you are using the FIFO.
    /// BDR threshold
    /// INT1, INT2


    set_accel_ODR(OUTPUT_DATA_RATES::ODR_833_HZ);
    set_accel_full_scale(ACCEL_FULL_SCALE::ACCEL_SCALE_32G);
    set_gyro_ODR(OUTPUT_DATA_RATES::ODR_833_HZ);
    set_gyro_full_scale(GYRO_FULL_SCALE::GYRO_SCALE_2000DPS);
    set_interrupts_active_low(false);
    set_SPI_as_3_wire(false);

    enable_auto_address_increment(true);
    enable_data_ready_mask(true);
    enable_i2c_interface(true);
    enable_gyro_LPF1(false);
    enable_rounding(true, true); // rounding??
    enable_accel_high_performance_mode(true); /// The cutoff value of the LPF1 output is ODR/2 when the accelerometer is in high-performance mode. This
                                              ///  value is equal to 700 Hz when the accelerometer is in low-power or normal mode.


    enable_gyro_high_performance_mode(true);
    gyro_LPF1_bandwidth(0b111);
    enable_gyro_high_pass_filter(true);
    set_gyro_high_pass_filter_cutoff(GYRO_HIGH_PASS_FILTER_CUTOFF::GYRO_HPFC_16_MHZ);

    enable_LPF2(true);
    accel_high_pass_selection(false); // lpf2
    set_accel_high_pass_or_LPF2_filter_cutoff(ACCEL_HP_OR_LPF2_CUTOFF::ODR_OVER_800);
    enable_accel_fast_settling_mode(true);














    return 0;
}


