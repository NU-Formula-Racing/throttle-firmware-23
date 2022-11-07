#include <Arduino.h>
#include <throttle.h>
#include <cstdlib> // Including to gain access to absolute value function

/**
 * @brief Construct a new Throttle:: Throttle object
 * 
 * @param can_frame 
 */
Throttle::Throttle()
{
    
};

/**
 * @brief Reads how pressed the accelerometer pedal is and returns it as a
 * percentage from 0-100%
 * 
 * @return float 
 */
float Throttle::ReadAcceleratorPress()
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
    brake_position = analogRead(19)*350/4095;
    left_accelerometer_position = analogRead(22)*100/4095;
    right_accelerometer_position = analogRead(18)*100/4095;
    if (brake_position > 0 && left_accelerometer_position > 0 && right_accelerometer_position > 0) {
        return true;
    }
    return false;
};
