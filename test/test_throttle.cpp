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

void DoPotentiometersAgree(void) {
    // Testing position values when they should agree with each other
    TEST_ASSERT_TRUE(DoPotentiometersAgree(70, 74));
    TEST_ASSERT_TRUE(DoPotentiometersAgree(74, 70));
    TEST_ASSERT_TRUE(DoPotentiometersAgree(70, 79));
    TEST_ASSERT_TRUE(DoPotentiometersAgree(79, 70));
    TEST_ASSERT_TRUE(DoPotentiometersAgree(0, 0));
    TEST_ASSERT_TRUE(DoPotentiometersAgree(100, 91));

    // Testing position values when they should NOT agree with each other
    TEST_ASSERT_FALSE(DoPotentiometersAgree(0, 11));
    TEST_ASSERT_FALSE(DoPotentiometersAgree(11, 0));
    TEST_ASSERT_FALSE(DoPotentiometersAgree(0, 100));
    TEST_ASSERT_FALSE(DoPotentiometersAgree(100, 0));
    TEST_ASSERT_FALSE(DoPotentiometersAgree(0, 10));

    // When position values differ by exactly 10% (should be false)
    TEST_ASSERT_FALSE(DoPotentiometersAgree(10, 0));
    TEST_ASSERT_FALSE(DoPotentiometersAgree(0, 10));
    TEST_ASSERT_FALSE(DoPotentiometersAgree(100, 90));
    TEST_ASSERT_FALSE(DoPotentiometersAgree(90, 100));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(SensorValueToPercentageTest);
    RUN_TEST(DoPotentiometersAgree);
    UNITY_END();
}