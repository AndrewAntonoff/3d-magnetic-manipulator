#ifndef VL53L5X_INTERFACE_H
#define VL53L5X_INTERFACE_H

#include "stm32h7xx_hal.h"

// Initialize the VL53L5X sensor (loads firmware, starts ranging)
uint8_t VL53L5X_Init(void);

// Read the distance in mm. Returns 0 if failed or 0 distance.
float VL53L5X_GetDistance(void);

// Task to poll data from sensor periodically
void VL53L5X_Process(void);

#endif // VL53L5X_INTERFACE_H
