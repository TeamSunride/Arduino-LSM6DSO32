#include <Arduino.h>
#include "LSM6DS032.h"


// TODO: README - photos etc
// TODO: examples folder


#define DEBUG Serial.printf("We got here: %s  Line:%d\n", __FILE__, __LINE__)

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
 * Datasheet on the finite state machine: https://www.st.com/resource/en/application_note/an5505-lsm6dso32-finite-state-machine-stmicroelectronics.pdf
 *
 * Application note: https://www.st.com/resource/en/application_note/dm00517282-lsm6dso-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf
*/

/* Usage */
#define CS_pin 10
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE2);
//LSM6DS032 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
LSM6DS032 LSM(&Wire, 1000000); // i2c protocol constructor


Fifo<Vector<double, 4>> accFifo(1024);
Fifo<Vector<double, 4>> gyrFifo(1024);
LSM_FIFO_STATUS fifo_status;

void setup() {
    delay(1000);
    Serial.begin(9600);
    Vector<double, 3> a;
    DEBUG;

    LSM.begin();
    DEBUG;
    LSM.default_configuration();
    DEBUG;

    fifo_status = LSM.get_fifo_status();
    LSM.fifo_clear();
    DEBUG;
}

Vector<double, 4> acc = {0,0,0,0};
void loop() {
    DEBUG;
    unsigned long start = micros();
    fifo_status = LSM.get_fifo_status();
    int num_unread = fifo_status.num_fifo_unread;
    if (Serial) Serial.printf("\nNum Unread: %d\n", num_unread);
    for (int i=0;i<num_unread;i++) {
        LSM.fifo_pop(accFifo, gyrFifo);
    }

    if (accFifo.fifo_status() != Fifo_STATUS::Fifo_EMPTY) {
        int usedSpace = accFifo.used_space();
        for (int i = 0; i < usedSpace; i++) {
            acc = accFifo.pop();
            if (Serial) Serial.printf("%lf, %lf, %lf\n", acc[0], acc[1], acc[2]);
        }
    }



    //Serial.printf("\nTime: %d\n", micros() - start);
    delayMicroseconds(5000-(micros()-start));
}
