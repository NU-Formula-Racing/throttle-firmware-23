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

    // right_acc_pos = analogRead(acc_sensor_right);
    // left_acc_pos = analogRead(acc_sensor_left);
    // brake_pos = analogRead(brake_sensor)

    //temp values (adjust when we get protoype because they should be to the left of deadzone)
    //MIN_VAL_RIGHT is the value from the right sensor when the driver is resting foot on pedal (determine from testing)
    //MAX_VAL_RIGHT is the value from the right sensor when the pedal is fully pressed
    const uint16_t MIN_VAL_RIGHT = 370;
    const uint16_t MAX_VAL_RIGHT = 1680;
    //define range for left sensor
    /*
    const int MIN_VAL_LEFT = some number;
    const int MAX_VAL_LEFT = some number;
    */


    /* When we have all 3 sensors:
    if (!brakeAndAccelerator() && arePotentiometersCorrect()) {
        float pedal_val = (right_acc_pos + left_acc_pos)/2;
    }
    else {
        return 0;
    }
    */

    //for now
    float pedal_val = max(analogRead(acc_sensor_right), MAX_VAL_RIGHT);

    //map sensor value to percent (0 to 100)
    float throttle_percent = (pedal_val-MIN_VAL_RIGHT)*100/(MAX_VAL_RIGHT-MIN_VAL_RIGHT); 

    //apply equation to convert the throttle's position (percent) to torque
    uint16_t torque = exp(0.06 * (throttle_percent - 9));
    if (torque > 230) {
        torque = 230;
    }

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
    float percent_dif = (abs(left_acc_pos - right_acc_pos) / ((left_acc_pos + right_acc_pos)/2)) * 100;
    if (percent_dif < 10) {
        return true;
    }
    return false;
};


bool Throttle::brakeAndAccelerator()
{
    // PSEUDOCODE
    /*
    The following are the values that the sensors display before any pedals are pressed
    float brake_push_val = some value
    float left_acc_val = some value
    float right_acc_val = some value
    if (brake_position > brake_push_val && left_acc_pos > left_acc_val && right_acc_pos > right_acc_val) {
        return true;
    }
    return false;
    */

   // TEMP UNTIL WE HAVE ALL SENSORS
   return false;
};
