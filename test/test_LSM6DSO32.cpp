//
// Created by robos on 23/06/2022.
//
#include <unity.h>
#include "LSM6DSO32.h"

#define CS_pin 10
#define WHO_AM_I_ID 0x6C


LSM6DSO32 LSMTest(CS_pin, SPI, 4000000);

void setUp(void) {
    // set stuff up here
    Serial.begin(115200);
    LSMTest.begin();
    LSMTest.default_configuration();
}

void tearDown(void) {
    // clean stuff up here
}

/* -------------------------------- Tests -------------------------------- */
void test_who_am_i() {
    TEST_ASSERT(LSMTest.who_am_i() == WHO_AM_I_ID);
}

void test_watermark_set() {
    short wtm = 511;
    LSMTest.set_fifo_watermark(wtm);
    TEST_ASSERT(LSMTest.get_fifo_watermark() == wtm);
}

void test_get_accel() {
    Vector<double, 3> acc = {0,0,0};
    for(int i=0; i<100; i++) {
        acc = LSMTest.get_accel();
        delay(1); // 1ms
    }
    TEST_ASSERT(acc.norm()!=0);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_who_am_i);
    RUN_TEST(test_watermark_set);
    RUN_TEST(test_get_accel);
    UNITY_END();
}