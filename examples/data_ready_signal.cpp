/*
 * An example of how the LSM6DSO32 can be used by reading directly, using the "data ready" signal.
 * The built-in Fifo is not being used.
 *
 */


#include <Arduino.h>
#include "LSM6DSO32.h"

// The LSM6DSO32 can be used with either SPI or I2C, and this library supports both, using Protocol: https://github.com/TeamSunride/Protocol
#define CS_pin 10
LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
// LSM6DSO32 LSM(&Wire, 1000000); // i2c protocol constructor


void setup() {
    // Setup Serial on 115200 baud
    Serial.begin(115200);

    // begin() initialises the communication protocol.
    LSM.begin();

    // default_configuration() configures the device. - Specific settings can be set after calling this:
    LSM.default_configuration();
    LSM.set_dataready_pulsed(false); // Ensure the signal is in "latched mode" not "pulsed".

    // Super low output rate
    LSM.enable_accel_ultra_low_power(true);
    LSM.set_accel_ODR(OUTPUT_DATA_RATES::ODR_1_6_HZ);

    // Very high output rate (uncomment to try)
//    LSM.enable_accel_ultra_low_power(false);
//    LSM.enable_accel_high_performance_mode(true);
//    LSM.set_accel_ODR(OUTPUT_DATA_RATES::ODR_6667_HZ);
}

void loop() {
    // directly read from the accel and gyro registers - not using the fifo
    while (LSM.get_accel_drdy_status() != 1); // wait for the signal.
    Vector<double, 3> acc = LSM.get_accel(); // read the data.
    Vector<double, 3> gyr = LSM.get_gyro();
    Serial.printf("Acc: %lf, %lf, %lf\n", acc[0], acc[1], acc[2]);
    //Serial.printf("Gyr: %lf, %lf, %lf\n", gyr[0], gyr[1], gyr[2]);

}
