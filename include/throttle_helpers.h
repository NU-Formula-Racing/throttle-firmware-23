#include <stdint.h>

/**
 * @brief Takes a sensor value (sensor_val), it's minimum value, and it's
 * maximum value to convert it to a percentage from 1 - 100
 * 
 * @param sensor_val Sensor value from analogRead
 * @param min_val Minimum value of sensor when driver is resting foot on pedal (determined experimentally)
 * @param max_val Maximum value of sensor when pedal is fully pressed (determined experimentally)
 * @return uint16_t Sensor value as a percentage from 1 - 100
 */
uint16_t SensorValueToPercentage(uint16_t sensor_val, uint16_t min_val, uint16_t max_val);

/**
 * @brief Returns true if the position values for the left accelerometer sensor
 * (left_acc_pos) and the right accelerometer sensor (right_acc_pos) do not
 * differ by more than 10%
 * 
 * @param left_acc_pos Position value for left accelerometer sensor (percentage from 0-100)
 * @param right_acc_pos Position value for right accelerometer sensor (percentage from 0-100)
 * @return true The position values agree with each other (less than 10% difference)
 * @return false The position values DON'T agree with each other
 */
bool DoPotentiometersAgree(uint16_t left_acc_pos, uint16_t right_acc_pos);

/**
 * @brief Converts throttle_percent to a corresponding torque value
 * 
 * @param throttle_percent Percentage measuring how much pedal is being pressed
 * (this is determined from both the position of the left accelerometer sensor
 * and the right accelerometer sensor)
 * @return uint16_t Torque
 */
uint16_t ConvertPositionToTorque(uint16_t throttle_percent);

/**
 * @brief Checks if the break and accelerator pedals are not pressed simultaneously.
 * 
 * @param brake_position The position of the break
 * @param throttle_percent Percentage measuring how much accelerator pedals pressed from 0-100
 * @return true 
 * @return false 
 */
bool BrakeAndAccelerator(uint16_t brake_position, uint16_t throttle_percent);

