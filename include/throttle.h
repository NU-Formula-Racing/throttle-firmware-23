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
        // Reads how much the accelerometer is being pressed (a percentage
        // from 0 to 100)
        float ReadAcceleratorPress();

        // Returns true if the two potentiometers on the acceleration pedal
        // agree with each other
        bool arePotentiometersCorrect();

        // Returns true if the brake and accelerator pedals are simultaneously pressed,
        // which would result in the throttle being set to 0 percent
        bool brakeAndAccelerator();

        void updateBrakePosition();

    private:
        long brake_position;
        // Both potentiometer voltages must be within a threshold
        long left_accelerometer_position;
        long right_accelerometer_position;
        // Will be communicated to the CAN bus
        long throttle_limit;
};
