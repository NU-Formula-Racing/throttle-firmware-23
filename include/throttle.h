#include <stdint.h>

/**
 * @brief Definitions and constants for the throttle.
 * 
 */
class Throttle
{
    public:
        // TODO: Maybe add constants
        /*
        static constexpr int throttleSensorPin =;
        */
        Throttle();

        // Reads how pressed the accelerometer pedal is and returns it as a
        // torque value
        uint16_t GetAcceleratorPress(float motor_temp, float batt_amp, float batt_voltage);

        // Updates fields of Throttle
        void updateValues();

        //  Takes a sensor value (sensor_val), it's minimum value, and it's maximum value to convert it to a percentage from 1 - 100
        uint16_t SensorValueToPercentage(uint16_t sensor_val, uint16_t min_val, uint16_t max_val);

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
