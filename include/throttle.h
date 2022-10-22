/**
 * @brief Definitions and constants for the throttle. Initialization?????
 * 
 */
class Throttle
{
    public:
        // Include Constants???
        /*
        static constexpr int throttleSensorPin = 2;
        */
        Throttle();
        float ReadAccelerometerPress();
        bool arePotentiometersCorrect();
        bool isbreakPedalPlausible();
        void updateThrottleLimit();

    private:
        long break_position;
        // Both potentiometer voltages must be within a threshold
        long left_accelerator_position;
        long right_accelerator_position;
        // Will be communicated to the CAN bus
        long throttle_limit;
};