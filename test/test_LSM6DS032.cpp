//
// Created by robos on 23/06/2022.
//
#include <unity.h>
#include <Arduino.h>
#include <SPI.h>
#include "Wire.h"
#include "LSM6DS032.h"

#define CS_pin 10
#define WHO_AM_I_ID 0x6C

SPISettings settings = SPISettings(4000000, MSBFIRST, SPI_MODE2);
LSM6DS032 LSM(CS_pin, SPI, settings);

void setUp(void) {
    // set stuff up here
    Serial.begin(9600);
    LSM.begin();
}

void tearDown(void) {
    // clean stuff up here
}

void test_watermark_set() {
    short wtm = 511;
    LSM.set_fifo_watermark(wtm);
    TEST_ASSERT(LSM.get_fifo_watermark() == wtm);
}

void test_who_am_i() {
    TEST_ASSERT(LSM.who_am_i() == WHO_AM_I_ID);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_who_am_i);
    RUN_TEST(test_watermark_set);
    UNITY_END();
}