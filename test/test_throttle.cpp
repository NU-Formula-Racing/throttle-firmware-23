#include "throttle.h"
#include <unity.h>
#include <stdint.h>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void sensorValueToPercentageTest(void) {
    // Testing current constants used in getAcceleratorPress()
    Throttle throttle;
    const uint16_t MIN_VAL_RIGHT = 370;
    const uint16_t MAX_VAL_RIGHT = 1680;

    // TEST_ASSERT_EQUAL(throttle.sensorValueToPercentage(MIN_VAL_RIGHT, MIN_VAL_RIGHT, MAX_VAL_RIGHT), 0);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    UNITY_END();
}