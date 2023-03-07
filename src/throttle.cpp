#include <Arduino.h>
#include <throttle.h>
#include <throttle_helpers.h>

#include <cmath>
#include <numeric>
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
    return static_cast<uint16_t>(throttle_perc * motor_perc * torque_perc);

    // apply equation to convert the throttle's position (percent) to torque
    // uint16_t torque = min(exp(0.06 * (throttle_percent - 9)), 230.0);
};

void Throttle::CalculateMovingAverage()
{
    const uint16_t ACC_SENSOR_LEFT = 35;
    uint16_t left_acc_val = analogRead(ACC_SENSOR_LEFT);
    if (leftvalues.size() < 10) {
        leftvalues.push_back(left_acc_val);
    } else {
        leftvalues.erase(leftvalues.begin());
        leftvalues.push_back(left_acc_val);
    }
    leftaverage = accumulate(leftvalues.begin(), leftvalues.end(), 0.0) / leftvalues.size();
    const uint16_t ACC_SENSOR_RIGHT = 36;
    uint16_t right_acc_val = analogRead(ACC_SENSOR_RIGHT);
    if (rightvalues.size() < 10) {
        rightvalues.push_back(right_acc_val);
    } else {
        rightvalues.erase(rightvalues.begin());
        rightvalues.push_back(right_acc_val);
    }
    rightaverage = accumulate(rightvalues.begin(), rightvalues.end(), 0.0) / rightvalues.size();
};

void Throttle::updateValues()
{
    // initialize pin values
    const uint16_t ACC_SENSOR_LEFT = 35;
    const uint16_t ACC_SENSOR_RIGHT = 36;
    const uint16_t BRAKE_SENSOR = 25;

    // temp values (adjust when we get protoype because they should be to the left of deadzone)
    // MIN_VAL_RIGHT is the value from the right sensor when the driver is resting foot on pedal (determine from
    // testing)

    // define range for left sensor and brake sensor (min_val will always be when foot is resting on pedal)
    const uint16_t MIN_VAL_LEFT = 1500;
    const uint16_t MAX_VAL_LEFT = 2000;

    // MAX_VAL_RIGHT is the value from the right sensor when the pedal is fully pressed
    // NOTE: Occasionally outlier values less than 1450 appear and over 2050 appear, though they are very uncommon. We will
    // treat 1450 as the right sensor's minimum value and 2050 as its maximum value for now.
    const uint16_t MIN_VAL_RIGHT = 1450;
    const uint16_t MAX_VAL_RIGHT = 2050;

    const uint16_t MIN_VAL_BRAKE = 0;
    const uint16_t MAX_VAL_BRAKE = 1680;


    // Serial.print("left_acc_val:");
    // Serial.println(analogRead(ACC_SENSOR_LEFT));
    // Serial.println(leftaverage);
    /*
    Serial.print("left_acc_pos:");
    Serial.println(left_acc_pos);
    */

    // Serial.print("right_acc_val:");
    // Serial.println(analogRead(ACC_SENSOR_RIGHT));
    // Serial.println(rightaverage);

    // Serial.print("brake value:");
    // Serial.println(analogRead(BRAKE_SENSOR));
    /*
    Serial.print("right_acc_pos:");
    Serial.println(right_acc_pos);
    */

   /*
    Serial.print("DoPotentiometersAgree:");
    Serial.println(DoPotentiometersAgree(left_acc_pos, right_acc_pos));
   /*

    // right_acc_val, left_acc_val, brake_val are exact values from sensor (analogRead)
    // right_acc_pos, left_acc_pos, brake_pos will be a value from 0 to 100 -> allows for comparison because sensors
    // have different ranges


    uint16_t left_acc_val = max(leftaverage, MIN_VAL_LEFT); // Don't get negative values
    left_acc_val = min(left_acc_val, MAX_VAL_LEFT); // Ensure maximum value is not exceeded
    left_acc_pos = SensorValueToPercentage(left_acc_val, MIN_VAL_LEFT, MAX_VAL_RIGHT);

    uint16_t right_acc_val = max(rightaverage, MIN_VAL_RIGHT);
    right_acc_val = min(right_acc_val, MAX_VAL_RIGHT);
    right_acc_pos = SensorValueToPercentage(right_acc_val, MIN_VAL_RIGHT, MAX_VAL_RIGHT);

    // TODO: Uncomment once brake sensor is hooked up
    uint16_t brake_val = max(analogRead(BRAKE_SENSOR), MIN_VAL_BRAKE);
    brake_val = min(brake_val, MAX_VAL_BRAKE);
    brake_pos = SensorValueToPercentage(brake_val, MIN_VAL_BRAKE, MAX_VAL_BRAKE);
    // Serial.print("brake pos:");
    // Serial.println(brake_pos);

    // When we have all 3 sensors:
    float throttle_percent = (right_acc_pos + left_acc_pos) / 2.0;
    if (BrakeAndAccelerator(brake_pos, throttle_percent) || !DoPotentiometersAgree(right_acc_pos, left_acc_pos))
    {
        throttle_percent = 0;
    }

   // If the potentiometers do not agree (their values are not within 10% of
   // each other), then we must send 0 torque
    // if (DoPotentiometersAgree(left_acc_pos, right_acc_pos)) {
    //     throttle_percent = (left_acc_pos+right_acc_pos) / 2;
    // }

    // Set throttle position sensor value (0-1)
    throttle_perc = throttle_percent / 100.0;

    // Serial.print("Do potentiometers agree:");
    // Serial.println(DoPotentiometersAgree(left_acc_pos, right_acc_pos));
    // Serial.println();
    // Serial.println(throttle_perc);

};

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
