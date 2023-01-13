/*
 * An example of how the LSM6DSO32 can be used by reading directly from the accel and gyro registers
 * The built-in Fifo is not being used.
 *
 */


#include <Arduino.h>
#include "LSM6DSO32.h"

// The LSM6DSO32 can be used with either SPI or I2C, and this library supports both, using Protocol: https://github.com/TeamSunride/Protocol
#define CS_pin 40
LSM6DSO32 sensor(CS_pin, SPI, 4000000); // spi protocol constructor
// LSM6DSO32 sensor(&Wire, 1000000); // i2c protocol constructor


void setup() {
    // Setup Serial on 115200 baud
    Serial.begin(115200);

    // begin() initialises the communication protocol.
    sensor.begin();

    // default_configuration() configures the device. - Specific settings can be set after calling this:
    sensor.default_configuration();
    // for example:
    // sensor.set_interrupts_active_low(false);
}

void loop() {
    // directly read from the accel and gyro registers - not using the fifo
    Vector<double, 3> acc = sensor.get_accel();
    Vector<double, 3> gyr = sensor.get_gyro();
    Serial.printf("Acc: %lf, %lf, %lf\n     Gyr: %lf, %lf, %lf\n", acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);

    // Be aware of the ODR (Output data rate) that is set - reading at a rate higher than this means you will be reading stale data when using direct reads.
    // Note: The ODR can be set in setup() using e.g.    sensor.set_accel_ODR(OUTPUT_DATA_RATES::ODR_833_HZ);    and     sensor.set_gyro_ODR(OUTPUT_DATA_RATES::ODR_833_HZ);
    delay(10);
}