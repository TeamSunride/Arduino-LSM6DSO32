//
// Created by robos on 23/06/2022.
//
#include <unity.h>
#include "LSM6DSO32.h"

#define CS_pin 40
#define WHO_AM_I_ID 0x6C


LSM6DSO32::LSM6DSO32 LSMTest(CS_pin, SPI, 12000000);

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
    TEST_ASSERT(LSMTest.who_am_i() == LSM6DSO32::WHO_AM_I_ID);
}

void test_get_accel() {
    Vector<double, 3> acc = {0,0,0};
    for(int i=0; i<100; i++) {
        acc = LSMTest.get_accel();
        delay(1); // 1ms
    }
    TEST_ASSERT(acc.norm()!=0);
}

void test_get_gyro() {
    Vector<double, 3> gyr = {0,0,0};
    for (int i=0;i<100; i++) {
        gyr = LSMTest.get_gyro();
        delay(1); // 1ms
    }
    TEST_ASSERT(gyr.norm() != 0); // technically could be, but highly unlikely due to the noise.
}

void test_get_temp() {
    short temp = LSMTest.get_temperature();
    TEST_ASSERT(temp != 0);
}

void test_get_timestamp() {
    unsigned long timestamp = LSMTest.get_timestamp();
    TEST_ASSERT(timestamp != 0);
}

void test_write_reg() {
    LSMTest.write_reg(LSM6DSO32::REGISTER::X_OFS_USR, 0x23);
    TEST_ASSERT(LSMTest.read_reg(LSM6DSO32::REGISTER::X_OFS_USR) == 0x23);
}

void test_watermark_set() {
    short wtm = 511;
    LSMTest.set_fifo_watermark(wtm);
    TEST_ASSERT(LSMTest.get_fifo_watermark() == wtm);
}

void test_BDR_CNT_SET() {
    short thr = 2047;
    LSMTest.set_BDR_counter_threshold(thr);
    TEST_ASSERT(LSMTest.get_BDR_counter_threshold() == thr);
}

void test_set_BDR() {
    LSMTest.set_batching_data_rates(LSM6DSO32::BATCHING_DATA_RATES::BDR_104Hz, LSM6DSO32::BATCHING_DATA_RATES::BDR_104Hz); // accel, gyro
    byte reg = LSMTest.read_reg(LSM6DSO32::REGISTER::FIFO_CTRL3);
    TEST_ASSERT(reg == (((LSM6DSO32::BATCHING_DATA_RATES::BDR_104Hz<<4) | LSM6DSO32::BATCHING_DATA_RATES::BDR_104Hz)));
}

void test_XL_self_test() {
    bool pass = LSMTest.accel_self_test();
    TEST_ASSERT_TRUE(pass);
}

void test_GY_self_test() {
    bool pass = LSMTest.gyro_self_test();
    TEST_ASSERT_TRUE(pass);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_who_am_i);
    RUN_TEST(test_get_accel);
    RUN_TEST(test_get_gyro);
    RUN_TEST(test_get_temp);
    RUN_TEST(test_get_timestamp);
    RUN_TEST(test_write_reg);

    RUN_TEST(test_watermark_set);
    RUN_TEST(test_BDR_CNT_SET);
    RUN_TEST(test_set_BDR);

    LSMTest.default_configuration();
    RUN_TEST(test_XL_self_test);
    RUN_TEST(test_GY_self_test);

    UNITY_END();
}