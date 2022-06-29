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
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
LSM6DS032 LSM(CS_pin, SPI, settings);
//LSM6DS032 LSM(&Wire, 1000000);
FIFO<Vector<double>> accFIFO(256);
FIFO<Vector<double>> gyrFIFO(256);


void setup() {
    Serial.begin(115200);

    LSM.begin();
    LSM.default_configuration();
}


void loop() {

    LSM.fifo_pop();

    delay(3);
}
