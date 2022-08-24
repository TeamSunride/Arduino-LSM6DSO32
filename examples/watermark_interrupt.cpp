#include <Arduino.h>
#include "LSM6DSO32.h"

#define CS_pin 10
SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE2);
LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
//LSM6DSO32 LSM(&Wire, 1000000); // i2c protocol constructor

Fifo<Vector<double, 4>> accFifo(1024);
Fifo<Vector<double, 4>> gyrFifo(1024);
LSM_FIFO_STATUS fifo_status;

#define WTM_INTERRUPT_PIN 3 // change to your needs.

Vector<double, 4> acc = {0,0,0,0};
Vector<double, 4> gyr = {0,0,0,0};
void interrupt_handler() {
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
                Serial.printf("Acc: %lf, %lf, %lf", acc[0], acc[1], acc[2]);
                //Serial.printf("Gyr: %lf, %lf, %lf", gyr[0], gyr[1], gyr[2]);
                Serial.println();
            }
        }
    }
}

void setup() {
    Serial.begin(115200);

    LSM.begin();

    /* Config */
    // We begin with default configuration
    LSM.default_configuration();

    // Let's set the batching rates to something different.
    LSM.set_batching_data_rates(BATCHING_DATA_RATES::BDR_208Hz, BATCHING_DATA_RATES::BDR_208Hz);

    // Setting the Fifo watermark.
    LSM.set_fifo_watermark(64);

    // Setting the interrupt on PIN1 to FIFO_TH (when the fifo level passes the watermark, the interrupt goes high)
    LSM.set_INT1_INTERRUPT(INTERRUPTS::FIFO_TH, true);

    // Attaching the interrupt.
    pinMode(WTM_INTERRUPT_PIN, INPUT_PULLDOWN);
    attachInterrupt(WTM_INTERRUPT_PIN, interrupt_handler, RISING);
}

void loop() {
    delay(10);
}