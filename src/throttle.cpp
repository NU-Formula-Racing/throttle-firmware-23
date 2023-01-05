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

    // TEMP TESTING CODE

    // Directly read voltage outputted by the accelerometer sensor wired to pin 34
    // and convert it to a floating point percentage from 0 to 100 (100 percent
    // indicates sensor is outputting maximum possible voltage).
    float vol_percent = (analogRead(34) * 100) / 4095.0;

    // Map the value of vol_percent to correspond with the lever attached
    // to the sensor. No matter what the value of vol_percent is, we need
    // to transform it such that the throttle position is 0 when the lever is in
    // its base position and the throttle position is 100 when the lever has
    // been maximally turned.

    // min and max determined from testing
    uint16_t min_throttle_vol = 41;
    uint16_t max_throttle_vol = 92; // usually switches between 91 and 92
    uint16_t tolerance = 10;

    uint16_t throttle_pos;
    // Map vol_percent to throttle_pos. Note that the voltage is inversely
    // related to the position of the lever. When the lever is at its base
    // position, vol_percent is at 41. As the lever is turned toward its
    // maximum position, vol_percent decreases to 0 before restarting at
    // 100. It then proceeds to decrease until the lever reaches its
    // maximum position. At this time, vol_percent will be around 91/92.
    // This makes the math look a little unintuitive unfortunately.
    if ((vol_percent >= min_throttle_vol) && (vol_percent < (max_throttle_vol - tolerance))) {
        throttle_pos = 0;
    } else if ((vol_percent <= max_throttle_vol) && (vol_percent > (min_throttle_vol + tolerance))) {
        throttle_pos = 100;
    } else {
        // The total range of voltages outputed by the sensor when the lever
        // is turned freely spans the distance between min_throttle_vol and 0
        // added to the distance between 100 and max_throttle_vol (this is
        // given by the equation below).
        uint16_t total_vol_range = min_throttle_vol + (100 - max_throttle_vol);
        // Case where 0 <= vol_percent < min_throttle_vol
        if (vol_percent < min_throttle_vol) {
            throttle_pos = (min_throttle_vol -  vol_percent) / total_vol_range * 100;
        } else { // Case where max_throttle_vol < vol_percent <= 100
            throttle_pos = (min_throttle_vol + (100 - vol_percent)) / total_vol_range * 100;
        }
    }

    // Apply equation to convert the throttle's position to torque
    uint16_t torque = exp(0.06 * (throttle_pos - 9));
    if (torque > 230) {
        torque = 230;
    }

    /*
    Serial.print("Voltage (FOR TESTING): ");
    Serial.println(vol_percent);
    Serial.print("Position: ");
    Serial.println(throttle_pos);
    Serial.println();
    */
   /*
    Serial.print("Throttle Position: ");
    Serial.println(throttle_pos);
    Serial.print("Torque: ");
    Serial.println(torque);
    Serial.println();
    */
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
