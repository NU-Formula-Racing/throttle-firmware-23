#include <Arduino.h>
#include <throttle.h>
#include <throttle_helpers.h>

#include <cmath>
/**
 * @brief Construct a new Throttle:: Throttle object
 *
 * @param can_frame
 */
Throttle::Throttle()
{
    long brake_pos = 0;
    long left_acc_pos = 0;
    long right_acc_pos = 0;
    float throttle_perc = 0;

    // Derivative to measure change in right_acc_pos
};

/**
 * @brief Reads how pressed the accelerometer pedal is and returns it as a
 * percent from 0-100
 *
 * @return uint16_t
 */
uint16_t Throttle::GetAcceleratorPress(float motor_temp, float batt_amp, float batt_voltage, float rpm)
{
    updateValues();
    float motor_perc = motorPercent(motor_temp);
    float torque_perc = min(convertBattAmp(batt_amp, batt_voltage, rpm), (float)100);
    return throttle_perc * motor_perc * torque_perc;

    // apply equation to convert the throttle's position (percent) to torque
    // uint16_t torque = min(exp(0.06 * (throttle_percent - 9)), 230.0);
};

void Throttle::updateValues()
{
    // initialize pin values
    const uint16_t ACC_SENSOR_RIGHT = 34;
    const uint16_t ACC_SENSOR_LEFT = 35;
    const uint16_t BRAKE_SENSOR = 36;

    // temp values (adjust when we get protoype because they should be to the left of deadzone)
    // MIN_VAL_RIGHT is the value from the right sensor when the driver is resting foot on pedal (determine from
    // testing) MAX_VAL_RIGHT is the value from the right sensor when the pedal is fully pressed
    const uint16_t MIN_VAL_RIGHT = 370;
    const uint16_t MAX_VAL_RIGHT = 1680;
    // define range for left sensor and brake sensor (min_val will always be when foot is resting on pedal)
    /*
    const int MIN_VAL_LEFT = some number;
    const int MAX_VAL_LEFT = some number;
    const int MIN_VAL_BRAKE = some number;
    const int MAX_VAL_BRAKE = some number;
    */

    // right_acc_val, left_acc_val, brake_val are exact values from sensor (analogRead)
    // right_acc_pos, left_acc_pos, brake_pos will be a value from 0 to 100 -> allows for comparison because sensors
    // have different ranges
    uint16_t right_acc_val = max(analogRead(ACC_SENSOR_RIGHT), MIN_VAL_RIGHT);  // Don't get negative values
    right_acc_val = min(right_acc_val, MAX_VAL_RIGHT);                          // Ensure maximum value is not exceeded
    right_acc_pos = SensorValueToPercentage(right_acc_val, MIN_VAL_RIGHT, MAX_VAL_RIGHT);

    // TODO: Uncomment once other sensors are hooked up
    /*
    uint16_t left_acc_val = max(analogRead(ACC_SENSOR_LEFT), MIN_VAL_LEFT);
    left_acc_pos = SensorValueToPercentage(left_acc_val, MIN_VAL_LEFT, MIN_VAL_RIGHT);
    uint16_t brake_val = max(analogRead(BRAKE_SENSOR), MIN_VAL_BRAKE);
    brake_pos = SensorValueToPercentage(brake_val, MIN_VAL_BRAKE, MIN_VAL_BRAKE);
    */

    // When we have all 3 sensors:
    /*
    float throttle_percent = (right_acc_pos + left_acc_pos) / 2.0;
    if (BrakeAndAccelerator(brake_pos, throttle_percent) || !DoPotentiometersAgree(right_acc_pos, left_acc_pos))
    {
        throttle_percent = 0;
    }
    */

    // for now
    float throttle_percent = right_acc_pos;

    // Set throttle position sensor value (0-1)
    throttle_perc = throttle_percent / 100.0;
}

float Throttle::convertBattAmp(float batt_amp, float batt_voltage, float rpm)
{
    // Use torque equation (includes power -> P = IV)
    uint8_t torque = 230;

    if (rpm != 0)
    {
        torque = 9.5488 * batt_amp * batt_voltage / rpm;
    }

    // map from 0 to 100 percent
    return torque * 100 / 230;
};

float Throttle::motorPercent(float motor_temp)
{
    if (motor_temp < 95)
    {
        return 1;
    }
    else if (motor_temp < 115)
    {
        return 1 - (1 / 20) * (motor_temp - 95);
    }
    else
    {
        return 0;
    }
};
