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
    long brake_position = 0;
    long left_accelerometer_position = 0;
    long right_accelerometer_position = 0;
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
    // PSEUDOCODE
    /*
    if (brakeAndAccelerator()) {
        return 0;
    }
    else if (this.arePotentiometersCorrect()) {
        left_accelerometer_position = analogRead(pin1)*100/4095; // pin1 and pin2 are temp labels
        right_accelerometer_position = analogRead(pin2)*100/4095;
        return (left_accelerator_position + right_acclerator_position)/2;
    }
    */

    const int acc_sensor1 = 34;

    float pedal_val = analogRead(acc_sensor1);
    //The following values assume the pedal goes from the middle knob (straight up) traveling down (counterclockwise when facing the brown side)
    //Cannot map from start to finish because there is a dead zone
    float max_val = 1680;
    float min_val = 370;
    float throttle_percent = (pedal_val-min_val)*100/(max_val-min_val);

    // Apply equation to convert the throttle's position to torque
    uint16_t torque = exp(0.06 * (throttle_percent - 9));
    if (torque > 230) {
        torque = 230;
    }
    if (pedal_val < 370) {
        throttle_percent = -100;
        torque = 0;
    }

    /*
    Serial.print("Voltage (FOR TESTING): ");
    Serial.println(vol_percent);
    Serial.print("Position: ");
    Serial.println(throttle_pos);
    Serial.println();
    */
    Serial.print("Throttle Position: ");
    Serial.println(throttle_percent);
    Serial.print("Torque: ");
    Serial.println(torque);
    Serial.println();
    return torque;
};

bool Throttle::arePotentiometersCorrect()
{
    // PSEUDOCODE
    int abs_dif = abs(left_accelerometer_position-right_accelerometer_position);
    float percent_dif = (abs_dif / ((left_accelerometer_position+right_accelerometer_position)/2)) * 100;
    if (percent_dif < 10) {
        return true;
    }
    return false;
};


bool Throttle::brakeAndAccelerator()
{
    // PSEUDOCODE: PIN VALUES ARE TEMP
    /*
    brake_position = analogRead(19)*350/4095;
    left_accelerometer_position = analogRead(22)*100/4095;
    right_accelerometer_position = analogRead(18)*100/4095;
    if (brake_position > 0 && left_accelerometer_position > 0 && right_accelerometer_position > 0) {
        return true;
    }
    return false;
    */

   // TEMP UNTIL WE HAVE ALL SENSORS
   return false;
};
