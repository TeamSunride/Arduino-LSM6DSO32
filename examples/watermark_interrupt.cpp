/*
 * An example of how the LSM6DSO32 can be used by using the watermark threshold interrupt.
 *
 * When the interrupt is received, the built-in FIFO is flushed.
 *
 */


#include <Arduino.h>
#include "LSM6DSO32.h"

#define CS_pin 10
LSM6DSO32::LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
//LSM6DSO32 LSM(&Wire, 1000000); // i2c protocol constructor

DynamicFifo<Vector<double, 4>> accFifo(1024);
DynamicFifo<Vector<double, 4>> gyrFifo(1024);
LSM6DSO32::FIFO_STATUS fifo_status;



#define WTM_INTERRUPT_PIN 3 // change to your needs.

Vector<double, 4> acc = {0,0,0,0};
Vector<double, 4> gyr = {0,0,0,0};
void interrupt_handler() {
    // fifo_status contains flags about the FIFO state, as well as how many unread 7-byte sets there are.
    fifo_status = LSM.get_fifo_status();

    int num_unread = fifo_status.num_fifo_unread;
    // pop the number of unread 7-byte sets from the LSM's built-in FIFO.
    for (int i=0;i<num_unread;i++) {
        LSM.fifo_pop(accFifo, gyrFifo); // Flush the data into accFifo and gyrFifo
    }

    if (accFifo.fifo_status() != Fifo_STATUS::Fifo_EMPTY) {
        // It's important to note that num_unread in the LSM FIFO is *not* the same as the number of accel/gyro readings
        //   you have obtained. a) because of compression, b) because of extra data like timestamps that get batched in the FIFO.
        int usedSpace = accFifo.used_space();
        for (int i = 0; i < usedSpace; i++) {
            acc = accFifo.pop();
            if (Serial) {
                Serial.print("Acc: ");
                Serial.print(acc[0]); Serial.print(", ");
                Serial.print(acc[1]); Serial.print(", ");
                Serial.print(acc[2]);

                //  Serial.print("Timestamp: ");
                // Serial.print(acc[3]); // The 4th field in the Vector is the timestamp.

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

    // Example of how you can disable the gyro from batching in the built-in Fifo. (saves space if you aren't using the gyro)
    LSM.set_batching_data_rates(LSM6DSO32::BATCHING_DATA_RATES::BDR_208Hz, LSM6DSO32::BATCHING_DATA_RATES::NO_BATCHING); // accel, gyro

    // Setting the Fifo watermark. Range: (0-511)
    LSM.set_fifo_watermark(64);

    // Setting the interrupt on PIN1 to FIFO_TH (when the fifo level passes the watermark, the interrupt goes high)
    LSM.set_INT1_INTERRUPT(LSM6DSO32::INTERRUPTS::FIFO_TH, true);

    // Attaching the interrupt. - Arduino
    pinMode(WTM_INTERRUPT_PIN, INPUT_PULLDOWN);
    attachInterrupt(WTM_INTERRUPT_PIN, interrupt_handler, RISING);
}

void loop() {
    delay(10); // Do some stuff
}
