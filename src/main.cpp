#include <Arduino.h>
#include "LSM6DSO32.h"


//#define DEBUG Serial.printf("We got here: %s  Line:%d\n", __FILE__, __LINE__)

/*
 * Datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 *
 * Datasheet on the finite state machine: https://www.st.com/resource/en/application_note/an5505-lsm6dso32-finite-state-machine-stmicroelectronics.pdf
 *
 * Application note: https://www.st.com/resource/en/application_note/dm00517282-lsm6dso-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf
*/


/* Usage */
#define CS_pin 10
LSM6DSO32::LSM6DSO32 LSM(CS_pin, SPI, 12000000); // spi protocol constructor
//LSM6DSO32 sensor(&Wire, 1000000); // i2c protocol constructor


DynamicFifo<Vector<double, 4>> accFifo(1024);
DynamicFifo<Vector<double, 4>> gyrFifo(1024);
LSM6DSO32::FIFO_STATUS fifo_status;

void setup() {
    delay(1000);
    Serial.begin(115200);

    LSM.begin();
    LSM.default_configuration();
    LSM.set_accel_ODR(LSM6DSO32::OUTPUT_DATA_RATES::ODR_6667_HZ);
    LSM.set_accel_high_pass_or_LPF2_filter_cutoff(LSM6DSO32::ACCEL_HP_OR_LPF2_CUTOFF::ODR_OVER_800);

    fifo_status = LSM.get_fifo_status();
    LSM.fifo_clear();
}

Vector<double, 4> acc = {0,0,0,0};
Vector<double, 4> gyr = {0,0,0,0};
void loop() {
    unsigned long start = micros();
    fifo_status = LSM.get_fifo_status();
    int num_unread = fifo_status.num_fifo_unread;
    for (int i=0;i<num_unread;i++) {
        LSM.fifo_pop(accFifo, gyrFifo);
    }

    if (accFifo.fifo_status() != Fifo_STATUS::Fifo_EMPTY) {
        int usedSpace = accFifo.used_space();
        for (int i = 0; i < usedSpace; i++) {
            acc = accFifo.pop();
            gyr = gyrFifo.pop();
            if (Serial) {
                // comment out to your needs.
                Serial.print("Acc: ");
                Serial.print(acc[0]); Serial.print(", ");
                Serial.print(acc[1]); Serial.print(", ");
                Serial.print(acc[2]);

//                Serial.print("Gyr: ");
//                Serial.print(gyr[0]); Serial.print(", ");
//                Serial.print(gyr[1]); Serial.print(", ");
//                Serial.print(gyr[2]); Serial.print(", ");

                Serial.println();
            }
        }
    }


    delayMicroseconds(5000-(micros()-start)); // wait a bit
}