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
    float cur_throttle_signal = 0;
};

/**
 * @brief Reads how pressed the accelerometer pedal is and returns it as a
 * percentage from 0-100%
 * 
 * @return uint16_t
 */
uint16_t Throttle::ReadAcceleratorPress()
{
    //initialize pin values
    const int acc_sensor_right = 34;
    // const int acc_sensor_left = some pin #
    // const int brake_sensor = some pin #

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
    uint16_t right_acc_val = max(analogRead(acc_sensor_right), MIN_VAL_RIGHT); //Don't get negative values
    right_acc_pos = (right_acc_val-MIN_VAL_RIGHT)*100/(MAX_VAL_RIGHT-MIN_VAL_RIGHT);
    /*
    uint16_t left_acc_val = max(analogRead(acc_sensor_left), MIN_VAL_LEFT); 
    left_acc_pos = (left_acc_val-MIN_VAL_LEFT)*100/(MAX_VAL_LEFT-MIN_VAL_LEFT);
    uint16_t brake_val = max(analogRead(brake_sensor), MIN_VAL_BRAKE); 
    brake_pos = (brake_val-MIN_VAL_BRAKE)*100/(MAX_VAL_BRAKE-MIN_VAL_BRAKE);
    */

    /* When we have all 3 sensors:
    if (!brakeAndAccelerator() && arePotentiometersCorrect()) {
        float throttle_percent = (right_acc_pos + left_acc_pos)/2;
    }
    else {
        return 0;
    }
    */

    //for now
    float throttle_percent = right_acc_pos;

    //apply equation to convert the throttle's position (percent) to torque
    uint16_t torque = min(exp(0.06 * (throttle_percent - 9)), 230.0);

    /*
    Serial.print("Voltage (FOR TESTING): ");
    Serial.println(pedal_val);
    Serial.print("Throttle Percent: ");
    Serial.println(throttle_percent);
    Serial.println();
    Serial.print("Torque: ");
    Serial.println(torque);
    Serial.println();
    */
    return torque;
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
    if (brake_position > 0 && left_acc_pos > 0 && right_acc_pos > 0) {
        return true;
    }
    return false;
    */

   // TEMP UNTIL WE HAVE ALL SENSORS
   return false;
};
