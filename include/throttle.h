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
        // Reads how much the accelerometer is being pressed (torque 0-230)
        uint16_t ReadAcceleratorPress(float motor_temp, float batt_amp, float batt_voltage);

        // Updates left, right, and brake postions, and throttle sensor position (0-1)
        void updateValues();

        // Returns true if the two potentiometers on the acceleration pedal
        // agree with each other
        bool arePotentiometersCorrect();

        // Returns true if the brake and accelerator pedals are simultaneously pressed,
        // which would result in the throttle being set to 0 percent
        bool brakeAndAccelerator();

        // Converts max battery amperage into a torque value
        float convertBattAmp (float batt_amp, float batt_volatge);

        // Converts motor temperature into a percentage of torque
        float motorPercent (float motor_temp);

        void updateBrakePosition();

    private:
        //// Might not need all of these private fields

        long brake_pos;
        // Both potentiometer voltages must be within a threshold
        long left_acc_pos;
        long right_acc_pos;
        // Will be communicated to the CAN bus
        uint16_t cur_throttle_signal;
        float throttle_perc;
};
