#include <Arduino.h>
#include <SPI.h>
#include "protocol.h"
#include "LSM6DS032_registers.h"
#include "LSM6DS032.h"
#include <Wire.h>
/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
*/

#define CS_pin 10
SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE2);
LSM6DS032 LSM(CS_pin, SPI, settings);

void setup() {
    Serial.begin(9600);
}


void loop() {
    delay(10);
}
