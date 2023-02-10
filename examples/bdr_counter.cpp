/*
 * An example of how the LSM6DSO32 can be used by reading the bdr counter flag.
 * The BDR Counter counts how many of a certain type of data (accel or gyro) have been read since the last fifo flush.
 *      It can sometimes be more useful than watermark, because watermark measures fifo level, which contains many data sources,
 *      compressed and uncompressed.
 * It can be used in many versatile ways. Here it is being used to trigger a fifo flush.
 *
 * Warning: if the Fifo gets full before the BDR threshold is reached, the fifo may overwrite data (in continuous mode)
 * Set the BDR threshold wisely, and keep an eye on the Fifo levels too.
 */



#include <Arduino.h>
#include "LSM6DSO32.h"

// The LSM6DSO32 can be used with either SPI or I2C, and this library supports both, using Protocol: https://github.com/TeamSunride/Protocol
#define CS_pin 10
LSM6DSO32::LSM6DSO32 LSM(CS_pin, SPI, 4000000); // spi protocol constructor
//LSM6DSO32 sensor(&Wire, 1000000); // i2c protocol constructor

// Using dynamically allocated Fifo: https://github.com/TeamSunride/Fifo
DynamicFifo<Vector<double, 4>> accFifo(1024);
DynamicFifo<Vector<double, 4>> gyrFifo(1024);
LSM6DSO32::FIFO_STATUS fifo_status;

void setup() {
    // Setup Serial on 115200 baud
    Serial.begin(115200);

    // begin() initialises the communication protocol.
    LSM.begin();

    // default_configuration() configures the device. - Specific settings can be set after calling this:
    LSM.default_configuration();
    // Example of how you can disable the gyro from batching in the built-in Fifo. (saves space if you aren't using the gyro)
    LSM.set_batching_data_rates(LSM6DSO32::BATCHING_DATA_RATES::BDR_833Hz, LSM6DSO32::BATCHING_DATA_RATES::NO_BATCHING); // accel, gyro
    LSM.enable_fifo_compression_runtime(true); // make sure compression is enabled.

    LSM.set_gyro_as_batch_count_trigger(false); // using accel as batch count trigger.
    // Set the BDR counter threshold. Range: (0-2047)
    LSM.set_BDR_counter_threshold(800);
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

    // When 800 accelerometer readings have been counted, flush the fifo.
    // Note that this is *not* the same as using watermark, because of the compression algorithm. The fifo level at 800 accel readings is about 420 out of 512.
    while (fifo_status.counter_bdr_flag != 1) {
        fifo_status = LSM.get_fifo_status();
    }

    int num_unread = fifo_status.num_fifo_unread;
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
    // Serial.printf("%d\n", num_unread);
}

