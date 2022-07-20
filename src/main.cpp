#include <Arduino.h>
#include <SPI.h>
#include "protocol.h"
#include "LSM6DS032_registers.h"
#include "LSM6DS032.h"
#include <Wire.h>
#include "Fifo.h" // dynamically allocated Version.


#define DEBUG Serial.printf("%s %d\n", __FILE__, __LINE__)

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
 * Datasheet on the finite state machine: https://www.st.com/resource/en/application_note/an5505-lsm6dso32-finite-state-machine-stmicroelectronics.pdf
 *
 * Application note: https://www.st.com/resource/en/application_note/dm00517282-lsm6dso-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf
*/

/* Usage */
#define CS_pin 10
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
LSM6DS032 LSM(CS_pin, SPI, settings); // spi protocol constructor
//LSM6DS032 LSM(&Wire, 1000000); // i2c protocol constructor

Fifo<Vector<double, 4>> accFifo(1024);
Fifo<Vector<double, 4>> gyrFifo(1024);
LSM_FIFO_STATUS fifo_status;

void setup() {
    delay(1000);

    Vector<double, 3> a;

    LSM.begin();
    LSM.default_configuration();

    fifo_status = LSM.get_fifo_status();
    delay(1000);

}


void loop() {
    unsigned long start = micros();
    fifo_status = LSM.get_fifo_status();
    int num_unread = fifo_status.num_fifo_unread;
    //Serial.printf("\nNum Unread: %d\n", num_unread);
    for (int i=0;i<num_unread;i++) {
        LSM.fifo_pop(accFifo, gyrFifo);
    }



    //Serial.printf("\nTime: %d\n", micros() - start);
    delayMicroseconds(10000-(micros()-start));
}
