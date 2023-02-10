#include <Arduino.h>
#include <throttle.h>
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
};

/**
 * @brief Reads how pressed the accelerometer pedal is and returns it as a
 * percent from 0-100
 * 
 * @return uint16_t
 */
uint16_t Throttle::GetAcceleratorPress(float motor_temp, float batt_amp, float batt_voltage)
{
    updateValues();
    float motor_perc = motorPercent(motor_temp);
    float torque_perc = min(convertBattAmp(batt_amp, batt_voltage), (float)100);
    return throttle_perc * motor_perc * torque_perc;

    // apply equation to convert the throttle's position (percent) to torque
    // uint16_t torque = min(exp(0.06 * (throttle_percent - 9)), 230.0);
};

void Throttle::updateValues()
{
    //initialize pin values
    const uint16_t ACC_SENSOR_RIGHT = 34;
    // const uint16_t ACC_SENSOR_LEFT = some pin #
    // const uint16_t BRAKE_SENSOR = some pin #

    //temp values (adjust when we get protoype because they should be to the left of deadzone)
    //MIN_VAL_RIGHT is the value from the right sensor when the driver is resting foot on pedal (determine from testing)
    //MAX_VAL_RIGHT is the value from the right sensor when the pedal is fully pressed
    const uint16_t MIN_VAL_RIGHT = 370;
    const uint16_t MAX_VAL_RIGHT = 1680;
    //define range for left sensor and brake sensor (min_val will always be when foot is resting on pedal)
    /*
    const int MIN_VAL_LEFT = some number;
    const int MAX_VAL_LEFT = some number;
    const int MIN_VAL_BRAKE = some number;
    const int MAX_VAL_BRAKE = some number;
    */

    //right_acc_val, left_acc_val, brake_val are exact values from sensor (analogRead)
    //right_acc_pos, left_acc_pos, brake_pos will be a value from 0 to 100 -> allows for comparison because sensors have different ranges
    uint16_t right_acc_val = max(analogRead(ACC_SENSOR_RIGHT), MIN_VAL_RIGHT); //Don't get negative values
    right_acc_pos = SensorValueToPercentage(right_acc_val, MIN_VAL_RIGHT, MAX_VAL_RIGHT);
    /*
    uint16_t left_acc_val = max(analogRead(ACC_SENSOR_LEFT), MIN_VAL_LEFT); 
    left_acc_pos = SensorValueToPercentage(left_acc_val, MIN_VAL_LEFT, MIN_VAL_RIGHT);
    uint16_t brake_val = max(analogRead(BRAKE_SENSOR), MIN_VAL_BRAKE); 
    brake_pos = SensorValueToPercentage(brake_val, MIN_VAL_BRAKE, MIN_VAL_BRAKE);
    */

    // When we have all 3 sensors:
    /*
    if (!brakeAndAccelerator() && arePotentiometersCorrect()) {
        float throttle_percent = (right_acc_pos + left_acc_pos)/2.0;
    }
    else {
        throttle_perc = 0;
    }
    */

    // for now
    float throttle_percent = right_acc_pos;

    // Set throttle position sensor value (0-1)
    throttle_perc = throttle_percent/100.0;
}

uint16_t Throttle::SensorValueToPercentage(uint16_t sensor_val, uint16_t min_val, uint16_t max_val)
{
    if (sensor_val < min_val) {
        return 0;
    }
    if (sensor_val > max_val) {
        return 100;
    }

    return (sensor_val-min_val)*100 / (max_val-min_val);
};

bool Throttle::arePotentiometersCorrect()
{
    // PSEUDOCODE
    if (abs(left_acc_pos - right_acc_pos) < 10) {
        return true;
    }
    return false;
};


bool Throttle::brakeAndAccelerator()
{
    // PSEUDOCODE
    /*
    if (brake_pos > 0 && left_acc_pos > 0 && right_acc_pos > 0) {
        return true;
    }
    return false;
    */

   // TEMP UNTIL WE HAVE ALL SENSORS
   return false;
};

float Throttle::convertBattAmp(float batt_amp, float batt_voltage)
{
    // Will get this value from the inverter
    // temp for now
    float rpm = 2.0;

    // Use torque equation (includes power -> P = IV)
    uint8_t torque = 9.5488 * batt_amp * batt_voltage / rpm;

    // map from 0 to 100 percent
    return torque * 100/230;
};

float Throttle::motorPercent(float motor_temp) 
{
    if (motor_temp < 95) {
        return 1;
    }
    else if (motor_temp < 115) {
        return 1-(1/20)*(motor_temp-95);
    }
    else {
        return 0;
    }
};
