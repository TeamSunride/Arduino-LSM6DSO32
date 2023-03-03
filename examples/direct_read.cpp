/*
 * An example of how the LSM6DSO32 can be used by reading directly from the accel and gyro registers
 * The built-in Fifo is not being used.
 *
 */


#include <Arduino.h>
#include "LSM6DSO32.h"

// The LSM6DSO32 can be used with either SPI or I2C, and this library supports both, using Protocol: https://github.com/TeamSunride/Protocol
#define CS_pin 40
LSM6DSO32::LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
// LSM6DSO32 sensor(&Wire, 1000000); // i2c protocol constructor


void setup() {
    // Setup Serial on 115200 baud
    Serial.begin(115200);

    // begin() initialises the communication protocol.
    LSM.begin();

    // default_configuration() configures the device. - Specific settings can be set after calling this:
    LSM.default_configuration();
    // for example:
    // LSM.set_interrupts_active_low(false);
}

void loop() {
    // directly read from the accel and gyro registers - not using the fifo
    Vector<double, 3> acc = LSM.get_accel();
    Vector<double, 3> gyr = LSM.get_gyro();
    Serial.print("Acc: ");
    Serial.print(acc[0]); Serial.print(", ");
    Serial.print(acc[1]); Serial.print(", ");
    Serial.print(acc[2]);

    Serial.print("Gyr: ");
    Serial.print(gyr[0]); Serial.print(", ");
    Serial.print(gyr[1]); Serial.print(", ");
    Serial.print(gyr[2]);

    // Be aware of the ODR (Output data rate) that is set - reading at a rate higher than this means you will be reading stale data when using direct reads.
    // Note: The ODR can be set in setup() using e.g.    LSM.set_accel_ODR(OUTPUT_DATA_RATES::ODR_833_HZ);    and     LSM.set_gyro_ODR(OUTPUT_DATA_RATES::ODR_833_HZ);
    delay(10);
}