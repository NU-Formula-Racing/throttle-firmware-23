#ifdef esp32dev

#include <stdint.h>

/**
 * @brief Definitions and constants for the throttle.
 * 
 */
class Throttle
{
    public:
        // Need to include constants
        /*
        static constexpr int throttleSensorPin =;
        */
        Throttle();

        // Reads how pressed the accelerometer pedal is and returns it as a
        // torque value
        uint16_t getAcceleratorPress();

        // Takes the value of a sensor, its minimum possible value, and maximum
        // possible value to convert it to a percentage from 0 to 100
        uint16_t sensorValueToPercentage(uint16_t sensorVal, uint16_t minVal, uint16_t maxVal);

        // Returns true if the two potentiometers on the acceleration pedal
        // agree with each other
        bool arePotentiometersCorrect();

        // Returns true if the brake and accelerator pedals are simultaneously pressed,
        // which would result in the throttle being set to 0 percent
        bool brakeAndAccelerator();

        void updateBrakePosition();

    private:
        //// Might not need all of these private fields

        long brake_pos;
        // Both potentiometer voltages must be within a threshold
        long left_acc_pos;
        long right_acc_pos;
        // Will be communicated to the CAN bus
        uint16_t cur_throttle_signal;
};

#endif