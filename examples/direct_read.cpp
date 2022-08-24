/*
 * An example of how the LSM6DSO32 can be used by reading directly from the registers
 * The built-in Fifo is not being used.
 *
 */


#include <Arduino.h>
#include "LSM6DSO32.h"

#define CS_pin 10
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE2);
LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
//LSM6DSO32 LSM(&Wire, 1000000); // i2c protocol constructor


void setup() {
    Serial.begin(115200);

    LSM.begin();
    LSM.default_configuration();

}

void loop() {
    // directly read from the accel and gyro registers - not using the fifo
    Vector<double, 3> acc = LSM.get_accel();
    Vector<double, 3> gyr = LSM.get_gyro();
    Serial.printf("Acc: %lf, %lf, %lf\n     Gyr: %lf, %lf, %lf\n", acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);
    delay(10);
}