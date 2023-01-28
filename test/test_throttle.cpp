#include "throttle_helpers.h"
#include <unity.h>
#include <stdint.h>
#include <iostream>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void SensorValueToPercentageTest(void) {
    // Testing current constants used in getAcceleratorPress()
    const uint16_t MIN_VAL_RIGHT = 370;
    const uint16_t MAX_VAL_RIGHT = 1680;

    TEST_ASSERT_EQUAL(SensorValueToPercentage(MIN_VAL_RIGHT, MIN_VAL_RIGHT, MAX_VAL_RIGHT), 0);
    TEST_ASSERT_EQUAL(SensorValueToPercentage(MAX_VAL_RIGHT, MIN_VAL_RIGHT, MAX_VAL_RIGHT), 100);
    TEST_ASSERT_EQUAL(SensorValueToPercentage((MIN_VAL_RIGHT + MAX_VAL_RIGHT)/2, MIN_VAL_RIGHT, MAX_VAL_RIGHT), 50);

    // If sensor_val outside of allowed range
    TEST_ASSERT_EQUAL(SensorValueToPercentage(0, MIN_VAL_RIGHT, MAX_VAL_RIGHT), 0);
    TEST_ASSERT_EQUAL(SensorValueToPercentage(10000, MIN_VAL_RIGHT, MAX_VAL_RIGHT), 100);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(SensorValueToPercentageTest);
    UNITY_END();
}