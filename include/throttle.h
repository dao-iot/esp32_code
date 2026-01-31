#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdint.h>

/**
 * @brief Initialize the throttle ADC module
 */
void throttle_init(void);

/**
 * @brief Read the current throttle percentage (0.0 to 100.0)
 * 
 * @return float Throttle percentage
 */
float throttle_get_percent(void);

#endif // THROTTLE_H
