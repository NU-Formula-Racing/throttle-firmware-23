/**
 * @brief Definitions and constants for the throttle. Initialization?????
 * 
 */
class Throttle
{
    public:
        // Need to include constants
        /*
        static constexpr int throttleSensorPin = 2;
        */
        Throttle();
        // Reads how much the accelerometer is being pressed (a percentage
        // from 0 to 100)
        float ReadAccelerometerPress();

        // Returns true if the two potentiometers on the acceleration pedal
        // agree with each other
        bool arePotentiometersCorrect();

        // Returns true if the break pedal measurements are plausible
        // (between 0 and 3.3 V)
        bool isbreakPedalPlausible();

        // Updates the throttle limit
        void updateThrottleLimit();

    private:
        long break_position;
        // Both potentiometer voltages must be within a threshold
        long left_accelerator_position;
        long right_accelerator_position;
        // Will be communicated to the CAN bus
        long throttle_limit;
};