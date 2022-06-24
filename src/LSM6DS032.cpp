//
// Created by robos on 23/06/2022.
//
#include "LSM6DS032.h"

/// Constructors
LSM6DS032::LSM6DS032(TwoWire *pipe, uint32_t freq) { // constructor for I2C protocol
    device = new I2CProtocol(LSM6DS032_DEFAULT_I2C_ADDRESS, pipe, freq);
}

LSM6DS032::LSM6DS032(byte chipSelect, SPIClass& spi, SPISettings settings) { // constructor for SPI protocol
    // TODO: is there a way to assert/ensure that the SPI settings are that of the LSM6DS032? - namely, MSB first, SPI mode 2.
    device = new SPIProtocol(chipSelect, spi, settings, READ_BYTE, WRITE_BYTE);
}

// Cheers GitHub copilot ;)

uint8_t LSM6DS032::enable_embedded_functions(bool enable) {
    byte data= device->read_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS);
    data= data& 0b11000000; // last 6 bits ' must be set to '0' for the correct operation of the device. ' according to the datasheet
    (enable) ? setBit(&data, 7, 1) : setBit(&data, 7, 0);
    return device->write_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS, data);
}

uint8_t LSM6DS032::enable_sensor_hub(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS);
    data = data & 0b11000000; // last 6 bits ' must be set to '0' for the correct operation of the device. ' according to the datasheet
    (enable) ? setBit(&data, 6, 1) : setBit(&data, 6, 0);
    return device->write_reg(LSM6DS032_REGISTER::FUNC_CFG_ADDRESS, data);
}

uint8_t LSM6DS032::enable_sdo_pullup(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::PIN_CTRL);
    setBit(&data, 7, 0);  data = data & 0b11000000; // essential for operation of the device
    (enable) ? setBit(&data, 6, 1) : setBit(&data, 6, 0);
    return device->write_reg(LSM6DS032_REGISTER::PIN_CTRL, data);
}

uint8_t LSM6DS032::set_fifo_watermark(short waterMarkBytes) { // waterMarkBytes should be a multiple of 7??
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
    (enable) ? setBit(&data, 7, 1) : setBit(&data, 7, 0);
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL2, data);
}

uint8_t LSM6DS032::enable_fifo_compression_runtime(bool enable) { // for enabling/disabling fifo compression at runtime
    // warning: does not check if FIFO_COMP_EN is set in EMB_FUNC_EN_B register.
    //          if FIFO_COMP_EN is not set, then the FIFO compression will not be enabled.
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL2);
    data = data & 0b11010111; // bits 3 and 5 must be zero.
    (enable) ? setBit(&data, 6, 1) : setBit(&data, 6, 0);
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL2, data);
}

uint8_t LSM6DS032::enable_fifo_compression(bool enable) {
    // TODO: enable_fifo_compression - requires embedded functions access.
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

uint8_t LSM6DS032::set_fifo_mode(FIFO_MODES mode) {
    byte data = device->read_reg(LSM6DS032_REGISTER::FIFO_CTRL4);
    data = data & 0b11110000; // bit 3 must be zero, clear bottom 3 bits
    data = mode | data;
    return device->write_reg(LSM6DS032_REGISTER::FIFO_CTRL4, data);
}

uint8_t LSM6DS032::set_dataready_pulsed(bool enable) {
    byte data = device->read_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1);
    data = data & 0b01100111; // bits 3 and 4 must be zero, clear bit 7
    (enable) ? setBit(&data, 7, 1) : setBit(&data, 7, 0);
    return device->write_reg(LSM6DS032_REGISTER::COUNTER_BDR_REG1, data);
}



