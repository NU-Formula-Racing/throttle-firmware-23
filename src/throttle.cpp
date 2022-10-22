#include <Arduino.h>
#include <throttle.h>

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
float Throttle::ReadAccelerometerPress()
{
    // Throttle constants

    // Function Body
};

bool Throttle::arePotentiometersCorrect()
{

};

bool Throttle::isbreakPedalPlausible()
{

};

void Throttle::updateThrottleLimit()
{

};

// NEED TO ADD ADDRESSES (IF NECESSARY)
enum CANFrameAddress
{
};