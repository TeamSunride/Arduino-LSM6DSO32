#include <Arduino.h>
#include "LSM6DSO32.h"



#define CS_pin 10
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE2);
LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
//LSM6DSO32 LSM(&Wire, 1000000); // i2c protocol constructor


Fifo<Vector<double, 4>> accFifo(1024);
Fifo<Vector<double, 4>> gyrFifo(1024);
LSM_FIFO_STATUS fifo_status;

void setup() {
    delay(1000);
    Serial.begin(115200);

    LSM.begin();
    LSM.default_configuration();
    LSM.set_fifo_watermark(64);

    fifo_status = LSM.get_fifo_status();
    LSM.fifo_clear();
}

Vector<double, 4> acc = {0,0,0,0};
Vector<double, 4> gyr = {0,0,0,0};

void loop() {
    fifo_status = LSM.get_fifo_status();
    while (fifo_status.watermark_flag != 1) {
        //delay(1);
        fifo_status = LSM.get_fifo_status();
    }

    int num_unread = fifo_status.num_fifo_unread;
    //if (Serial) Serial.printf("\nNum Unread: %d\n", num_unread);
    for (int i=0;i<num_unread;i++) {
        LSM.fifo_pop(accFifo, gyrFifo);
    }

    if (accFifo.fifo_status() != Fifo_STATUS::Fifo_EMPTY) {
        int usedSpace = accFifo.used_space();
        for (int i = 0; i < usedSpace; i++) {
            acc = accFifo.pop();
            gyr = gyrFifo.pop();
            if (Serial) {
                Serial.printf("Acc: %lf, %lf, %lf", acc[0], acc[1], acc[2]);
                Serial.printf("Gyr: %lf, %lf, %lf", gyr[0], gyr[1], gyr[2]);
                Serial.println();
            }
        }
    }
}

