#include <stdint.h>
#include <vector>

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
        uint16_t GetAcceleratorPress(float motor_temp, float batt_amp, float batt_voltage, float rpm);

        // Calculates moving average
        void CalculateMovingAverage();

        // Updates fields of Throttle
        void updateValues();

        // Converts max battery amperage into a torque value
        float convertBattAmp (float batt_amp, float batt_voltage, float rpm);

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
        std::vector<uint16_t> leftvalues;
        uint16_t leftaverage;
        std::vector<uint16_t> rightvalues;
        uint16_t rightaverage;
};
