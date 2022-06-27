#include <Arduino.h>
#include <SPI.h>
#include "protocol.h"
#include "LSM6DS032_registers.h"
#include "LSM6DS032.h"
#include <Wire.h>
#include "Vector.h"

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
 * Datasheet on the finite state machine: https://www.st.com/resource/en/application_note/an5505-lsm6dso32-finite-state-machine-stmicroelectronics.pdf
*/

/* Usage */
#define CS_pin 10
//SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
//LSM6DS032 LSM(CS_pin, SPI, settings);
//LSM6DS032 LSM(&Wire, 1000000);
void setup() {
    Serial.begin(9600);

//    LSM.begin();
//    LSM.default_configuration();
}


void loop() {
//    Vector<double> acc = LSM.get_accel();
//    Serial.printf("%lf, %lf, %lf\n", acc[0], acc[1], acc[2]);
//    delay(1);
}
