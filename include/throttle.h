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
    uint16_t GetThrottlePercent(float motor_temp, float batt_amp, float batt_voltage, float rpm);

    // Calculates moving average
    void CalculateMovingAverage();

    // mappings
    float lin_throttle_perc(float throttle_perc);
    float expon_throttle_perc(float throttle_perc);
    float log_throttle_perc(float throttle_perc);
    float bens_special_throttle_perc(float throttle_perc);

    // Updates fields of Throttle
    void updateValues();

    bool PotentiometersAgree();

    // Converts max battery amperage into a torque value
    float convertBattAmp(float batt_amp, float batt_voltage, float rpm);

    // Converts motor temperature into a percentage of torque
    float motorPercent(float motor_temp);

    bool brakePressed();

    uint8_t GetBrakePercentage();

    uint16_t GetLeftAccPos();

    uint16_t GetRightAccPos();

    uint16_t GetAccPos();

    uint8_t GetMaxAvailableTorquePercent();

    bool to3V3orGND();

    bool potentiometer_fault = 0;

    bool gnd_3v3_fault = 0;

    bool release_accel_fault = 0;

    void UpdateFaults();
    uint16_t leftaverage;
    uint16_t rightaverage;
    uint16_t brakeaverage;

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

    std::vector<uint16_t> rightvalues;

    std::vector<uint16_t> brakevalues;

    uint8_t max_available_torque_perc;
    bool wasBrakePressed;
    const uint8_t max_torque = 230;
    const uint16_t ACC_SENSOR_LEFT = 34;
    const uint16_t MIN_VAL_LEFT = 2770;
    const uint16_t MAX_VAL_LEFT = 3430;
    const uint16_t ACC_SENSOR_RIGHT = 35;
    const uint16_t MIN_VAL_RIGHT = 2640;
    const uint16_t MAX_VAL_RIGHT = 3540;
    const uint16_t BRAKE_SENSOR = 39;
    const uint16_t MIN_VAL_BRAKE = 2200;
    const uint16_t MAX_VAL_BRAKE = 3260;
    const uint16_t brakethreshold = 3500;
    const uint16_t leftthreshold = 3600;
    const uint16_t rightthreshold = 3800;
    const uint16_t brakeGND = 1000;
    const float max_motor_amp = 325;
};
