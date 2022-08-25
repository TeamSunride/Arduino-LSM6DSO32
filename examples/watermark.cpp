/*
 * An example of how the LSM6DSO32 can be used by reading the watermark threshold flag.
 *
 *
 */



#include <Arduino.h>
#include "LSM6DSO32.h"

// The LSM6DSO32 can be used with either SPI or I2C, and this library supports both, using Protocol: https://github.com/TeamSunride/Protocol
#define CS_pin 10
LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
//LSM6DSO32 LSM(&Wire, 1000000); // i2c protocol constructor

// Using dynamically allocated Fifo: https://github.com/TeamSunride/Fifo
Fifo<Vector<double, 4>> accFifo(1024);
Fifo<Vector<double, 4>> gyrFifo(1024);
LSM_FIFO_STATUS fifo_status;

void setup() {
    // Setup Serial on 115200 baud
    Serial.begin(115200);

    // begin() initialises the communication protocol.
    LSM.begin();

    // default_configuration() configures the device. - Specific settings can be set after calling this:
    LSM.default_configuration();
    // Example of how you can disable the gyro from batching in the built-in Fifo. (saves space if you aren't using the gyro)
    LSM.set_batching_data_rates(BATCHING_DATA_RATES::BDR_833Hz, BATCHING_DATA_RATES::NO_BATCHING); // accel, gyro

    // Set the fifo watermark. Range: (0-511)
    LSM.set_fifo_watermark(64);

    // Update the LSM's FIFO Status
    fifo_status = LSM.get_fifo_status();

    // Clear the LSM FIFO
    LSM.fifo_clear();
}

Vector<double, 4> acc = {0,0,0,0};
Vector<double, 4> gyr = {0,0,0,0};
void loop() {
    fifo_status = LSM.get_fifo_status();
    // fifo_status contains flags about the FIFO state, as well as how many unread 7-byte sets there are.
    while (fifo_status.watermark_flag != 1) { // Wait until the watermark flag is set high (FIFO level is > watermark.
        fifo_status = LSM.get_fifo_status();
    }

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
                Serial.printf("Acc: %lf, %lf, %lf", acc[0], acc[1], acc[2]);
                //Serial.printf("Timestamp: %lf   ", acc[4]); // The 4th field in the Vector is the timestamp.
                Serial.println();
            }
        }
    }
}

