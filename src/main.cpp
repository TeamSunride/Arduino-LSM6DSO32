#include <Arduino.h>
#include <SPI.h>
#include "device.h"
#include "LSM6DS032_registers.h"
#include "LSM6DS032.h"
#include <Wire.h>
/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
*/

#define CS_pin 10

LSM6DS032 LSM(I2C_SELECT, CS_pin, SPI_CHANNEL_0);

void setup() {
    // LSM uses SPI Mode 2 : Those lines are driven at the falling edge of SPC and should be captured at the rising edge of SPC.
    Serial.begin(9600);
}


void loop() {
    delay(10);
}
