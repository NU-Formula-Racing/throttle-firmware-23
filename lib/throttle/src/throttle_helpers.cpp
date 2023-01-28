#include <throttle_helpers.h>
#include <cmath>
#include <unity.h>
#include <stdint.h>

/**
 * @brief After using analogRead to read sensor value, use this function to convert
 * it into a percentage
 * 
 * TODO: Throw error messages if sensor_val less than min_val or greater
 * than max_val.
 * TODO: Throw error message if sensor_val somehow less than 0
 */
uint16_t SensorValueToPercentage(uint16_t sensor_val, uint16_t min_val, uint16_t max_val)
{
    if (sensor_val < min_val) {
        return 0;
    }
    if (sensor_val > max_val) {
        return 100;
    }

    return (sensor_val-min_val)*100 / (max_val-min_val);
}

/**
 * @brief Use to determine if there is a discrepancy by the values reported
 * by the left and right accelerometer position values (these are percentages)
 * 
 * TODO: Throw error messages if position values are less than 0 or greater
 * than 100
 */
bool DoPotentiometersAgree(uint16_t left_acc_pos, uint16_t right_acc_pos)
{
    return abs(left_acc_pos-right_acc_pos) < 10;
};