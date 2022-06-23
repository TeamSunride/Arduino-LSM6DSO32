#include <Arduino.h>
#include <SPI.h>
#include "device.h"
#include "LSMDS032_registers.h"
/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
*/

#define CS 10
I2CDevice sensor(0x6A, &Wire);

void setup() {
    // LSM uses SPI Mode 2 : Those lines are driven at the falling edge of SPC and should be captured at the rising edge of SPC.
    Serial.begin(9600);
}

void loop() {
    byte a = sensor.read_reg(OUTX_H_A);
    Serial.printf("%d\n", a);
}
