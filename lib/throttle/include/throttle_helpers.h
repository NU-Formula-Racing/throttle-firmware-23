#include <stdint.h>

/**
 * @brief Takes a sensor value (sensorVal), it's minimum value, and it's
 * maximum value to convert it to a percentage(1 - 100)
 * 
 * @return uint16_t 
 */
uint16_t SensorValueToPercentage(uint16_t sensorVal, uint16_t minVal, uint16_t maxVal);