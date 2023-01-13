/*
 * An example of how the LSM6DSO32 can be used by utilising the built-in FIFO.
 * The Fifo is checked every 5ms and flushed into the accFifo and gyrFifo after processing.
 *
 */


#include <Arduino.h>
#include "LSM6DSO32.h"

// The LSM6DSO32 can be used with either SPI or I2C, and this library supports both, using Protocol: https://github.com/TeamSunride/Protocol
#define CS_pin 10
LSM6DSO32::LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
//LSM6DSO32 LSM(&Wire, 1000000); // i2c protocol constructor


// Using dynamically allocated Fifo: https://github.com/TeamSunride/Fifo
Fifo<Vector<double, 4>> accFifo(1024);
Fifo<Vector<double, 4>> gyrFifo(1024);
LSM6DSO32::FIFO_STATUS fifo_status;

void setup() {
    // Setup Serial on 115200 baud
    Serial.begin(115200);

    // begin() initialises the communication protocol.
    LSM.begin();

    // default_configuration() configures the device. - Specific settings can be set after calling this:
    LSM.default_configuration();
    // Setting the fifo batching rate to something more appropriate.
    LSM.set_batching_data_rates(LSM6DSO32::BATCHING_DATA_RATES::BDR_833Hz, LSM6DSO32::BATCHING_DATA_RATES::BDR_833Hz); // Accel, gyro

    // To disable compression: (compression recommended)
    // LSM.enable_fifo_compression_runtime(false);

    fifo_status = LSM.get_fifo_status();
    LSM.fifo_clear();
}

Vector<double, 4> acc = {0,0,0,0};
Vector<double, 4> gyr = {0,0,0,0};
void loop() {
    unsigned long start = micros();
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
                Serial.printf("Acc: %lf, %lf, %lf     ", acc[0], acc[1], acc[2]);
                //Serial.printf("Timestamp: %lf   ", acc[4]); // The 4th field in the Vector is the timestamp.
                Serial.println();
            }
        }
    }
//    if (gyrFifo.fifo_status() != Fifo_STATUS::Fifo_EMPTY) {
//        int usedSpace = gyrFifo.used_space();
//        for (int i = 0; i < usedSpace; i++) {
//            gyr = gyrFifo.pop();
//            if (Serial) {
//                Serial.printf("Gyr: %lf, %lf, %lf     ", gyr[0], gyr[1], gyr[2]);
//                //Serial.printf("Timestamp: %lf   ", gyr[4]); // The 4th field in the Vector is the timestamp.
//                Serial.println();
//            }
//        }
//    }

    delayMicroseconds(5000-(micros()-start)); // wait for exactly 5ms
}
