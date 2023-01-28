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

