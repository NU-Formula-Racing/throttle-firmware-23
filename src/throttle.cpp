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
    uint8_t max_torque = 230;
    uint8_t max_available_torque = 100;
    bool wasBrakePressed = false;

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
    float max_motor_amp = 325;
    float max_motor_torque_perc = .94 * max_motor_amp / max_torque;
    float motor_perc = motorPercent(motor_temp);
    float torque_perc = min(convertBattAmp(batt_amp, batt_voltage, rpm), max_motor_torque_perc);
    // Serial.println(motor_perc);
    // Serial.println(torque_perc);
    max_available_torque = 100*motor_perc * torque_perc;
    return min(static_cast<uint16_t>(throttle_perc * motor_perc * torque_perc),static_cast<uint16_t>(100));

    // apply equation to convert the throttle's position (percent) to torque
    // uint16_t torque = min(exp(0.06 * (throttle_percent - 9)), 230.0);
};

void Throttle::CalculateMovingAverage()
{
    const uint16_t ACC_SENSOR_LEFT = 34;
    uint16_t left_acc_val = analogRead(ACC_SENSOR_LEFT);
    if (leftvalues.size() < 10) {
        leftvalues.push_back(left_acc_val);
    } else {
        leftvalues.erase(leftvalues.begin());
        leftvalues.push_back(left_acc_val);
    }
    leftaverage = accumulate(leftvalues.begin(), leftvalues.end(), 0.0) / leftvalues.size();
    
    const uint16_t ACC_SENSOR_RIGHT = 35;
    uint16_t right_acc_val = analogRead(ACC_SENSOR_RIGHT);
    if (rightvalues.size() < 10) {
        rightvalues.push_back(right_acc_val);
    } else {
        rightvalues.erase(rightvalues.begin());
        rightvalues.push_back(right_acc_val);
    }
    rightaverage = accumulate(rightvalues.begin(), rightvalues.end(), 0.0) / rightvalues.size();

    const uint16_t BRAKE_SENSOR = 39;
    uint16_t brake_val = analogRead(BRAKE_SENSOR);
    if (brakevalues.size() < 10) {
        brakevalues.push_back(brake_val);
    } else {
        brakevalues.erase(brakevalues.begin());
        brakevalues.push_back(brake_val);
    }
    brakeaverage = accumulate(brakevalues.begin(), brakevalues.end(), 0.0) / brakevalues.size();
};

// mappings
float Throttle::lin_throttle_perc(float throttle_perc)
{
    return throttle_perc;
};

float Throttle::expon_throttle_perc(float throttle_perc)
{
    if(throttle_perc <= 0.05 ){
        return 0;
    }
    else if(throttle_perc >= 0.05 && throttle_perc <= 1){
        return (-1.1245 * throttle_perc * throttle_perc + 2.23369 * throttle_perc - 0.10918);
    }
    else{
        return 1;
    }
};

float Throttle::log_throttle_perc(float throttle_perc)
{
    if(throttle_perc <= 0.05 ){
        return 0;
    }
    else if(throttle_perc >= 0.05 && throttle_perc <= 1){
        return (1.42401 * log10(4.27866 * throttle_perc + 0.776984));
    }
    else{
        return 1;
    }
};

float Throttle::bens_special_throttle_perc(float throttle_perc)
{
    if(throttle_perc < 0.05){
        return 0;
    }
    else if(throttle_perc >= 0.05 && throttle_perc < 0.15){
        return (8 * throttle_perc * throttle_perc + 0.4 * throttle_perc - 0.04); //exponetial section
    }
    else if(throttle_perc >= 0.15 && throttle_perc < 0.3){
        return (2.634 * throttle_perc - 0.1951); // linear section
    }
    else if(throttle_perc >= 0.3 && throttle_perc <= 0.98){
        return (0.7878 * log10(18.85 * throttle_perc + 0.0381)); // log section
    }
    else{
        return 1;
    }
};

void Throttle::updateValues()
{
    // // initialize pin values
    // const uint16_t ACC_SENSOR_LEFT = 34;
    // const uint16_t ACC_SENSOR_RIGHT = 35;
    // const uint16_t BRAKE_SENSOR = 39;

    // // temp values (adjust when we get protoype because they should be to the left of deadzone)
    // // MIN_VAL_RIGHT is the value from the right sensor when the driver is resting foot on pedal (determine from
    // // testing)

    // // define range for left sensor and brake sensor (min_val will always be when foot is resting on pedal)
    // const uint16_t MIN_VAL_LEFT = 1500;
    // const uint16_t MAX_VAL_LEFT = 2000;

    // // MAX_VAL_RIGHT is the value from the right sensor when the pedal is fully pressed
    // // NOTE: Occasionally outlier values less than 1450 appear and over 2050 appear, though they are very uncommon. We will
    // // treat 1450 as the right sensor's minimum value and 2050 as its maximum value for now.
    // const uint16_t MIN_VAL_RIGHT = 1450;
    // const uint16_t MAX_VAL_RIGHT = 2050;

    // const uint16_t MIN_VAL_BRAKE = 1700;
    // const uint16_t MAX_VAL_BRAKE = 1900;

    // // right_acc_val, left_acc_val, brake_val are exact values from sensor (analogRead)
    // // right_acc_pos, left_acc_pos, brake_pos will be a value from 0 to 100 -> allows for comparison because sensors
    // // have different ranges


    // uint16_t left_acc_val = max(leftaverage, MIN_VAL_LEFT); // Don't get negative values
    // left_acc_val = min(left_acc_val, MAX_VAL_LEFT); // Ensure maximum value is not exceeded
    // left_acc_pos = SensorValueToPercentage(left_acc_val, MIN_VAL_LEFT, MAX_VAL_LEFT);

    // uint16_t right_acc_val = max(rightaverage, MIN_VAL_RIGHT);
    // right_acc_val = min(right_acc_val, MAX_VAL_RIGHT);
    // right_acc_pos = SensorValueToPercentage(right_acc_val, MIN_VAL_RIGHT, MAX_VAL_RIGHT);

    // // TODO: Uncomment once brake sensor is hooked up
    // uint16_t brake_val = max(brakeaverage, MIN_VAL_BRAKE);
    // brake_val = min(brake_val, MAX_VAL_BRAKE);
    // brake_pos = SensorValueToPercentage(brake_val, MIN_VAL_BRAKE, MAX_VAL_BRAKE);

    // When we have all 3 sensors:
    // If the potentiometers do not agree (their values are not within 10% of
    // each other) or the brake is pressed, then we must send 0 torque
    left_acc_pos = GetLeftAccPos();
    right_acc_pos = GetRightAccPos();
    brake_pos = GetBrakePercentage();
    
    uint16_t throttle_percent = GetAccPos();
    
    if (BrakeAndAccelerator(brake_pos, throttle_percent)) {
        wasBrakePressed = true;
    }

    if (wasBrakePressed) {
        if (throttle_percent > 5) {
            throttle_percent = 0;
        } else {
            wasBrakePressed = false;
        }
    }

    if (!DoPotentiometersAgree(left_acc_pos, right_acc_pos) || to3V3orGND()) {
        throttle_percent = 0;
    }

    // Set throttle position sensor value (0-1)
    throttle_perc = 100 * bens_special_throttle_perc(throttle_percent / 100.0);
};

bool Throttle::PotentiometersAgree()
{
    return DoPotentiometersAgree(left_acc_pos, right_acc_pos);
}

float Throttle::convertBattAmp(float batt_amp, float batt_voltage, float rpm)
{
    // Use torque equation (includes power -> P = IV)
    float torque = 230;

    if (rpm != 0)
    {
        torque = 9.5488 * batt_amp * batt_voltage / rpm;
    }

    // map from 0 to 1
    return torque / static_cast<float>(max_torque);
};

float Throttle::motorPercent(float motor_temp)
{
    if (motor_temp < 95)
    {
        return 1;
    }
    else if (motor_temp < 115)
    {
        return 1 - (1 / 20.0) * (motor_temp - 95);
    }
    else
    {
        return 0;
    }
};

bool Throttle::brakePressed()
{
    return brake_pos > 5;
};

uint8_t Throttle::GetBrakePercentage()
{
    const uint16_t BRAKE_SENSOR = 39;
    const uint16_t MIN_VAL_BRAKE = 2300;
    const uint16_t MAX_VAL_BRAKE = 2500;
    uint16_t brake_val = max(brakeaverage, MIN_VAL_BRAKE);
    brake_val = min(brake_val, MAX_VAL_BRAKE);
    brake_pos = SensorValueToPercentage(brake_val, MIN_VAL_BRAKE, MAX_VAL_BRAKE);
    return brake_pos;
}

uint16_t Throttle::GetLeftAccPos()
{
    const uint16_t ACC_SENSOR_LEFT = 34;
    const uint16_t MIN_VAL_LEFT = 1300;
    const uint16_t MAX_VAL_LEFT = 1630;
    uint16_t left_acc_val = max(leftaverage, MIN_VAL_LEFT);
    left_acc_val = min(left_acc_val, MAX_VAL_LEFT);
    uint16_t left_acc_perc = SensorValueToPercentage(left_acc_val, MIN_VAL_LEFT, MAX_VAL_LEFT);
    return left_acc_perc;
}

uint16_t Throttle::GetRightAccPos()
{
    const uint16_t ACC_SENSOR_RIGHT = 35;
    const uint16_t MIN_VAL_RIGHT = 1490;
    const uint16_t MAX_VAL_RIGHT = 1930;
    uint16_t right_acc_val = max(rightaverage, MIN_VAL_RIGHT);
    right_acc_val = min(right_acc_val, MAX_VAL_RIGHT);
    uint16_t right_acc_perc = SensorValueToPercentage(right_acc_val, MIN_VAL_RIGHT, MAX_VAL_RIGHT);
    return right_acc_perc;
}

uint16_t Throttle::GetAccPos()
{
    uint16_t acc_perc = (GetLeftAccPos() + GetRightAccPos()) / 2;
    return acc_perc;
}

uint8_t Throttle::GetMaxAvailableTorque()
{
    return max_available_torque;
}

// Check if APPS1, APPS2, or brake signal goes to 3V3 (if goes to GND, 0 throttle automatically)
bool Throttle::to3V3orGND()
{
    uint16_t brakethreshold = 3000;
    uint16_t leftthreshold = 2500;
    uint16_t rightthreshold = 2500;
    uint16_t brakeGND = 1500;
    return (brakeaverage > brakethreshold || leftaverage > leftthreshold || rightaverage > rightthreshold || brakeaverage < brakeGND);
}