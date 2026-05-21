#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <string.h>
#include "i2c.h"

// Define maximum I2C transfer size
#define VL53L5CX_MAX_I2C_TRANSFER_SIZE 512

#define VL53L5CX_NB_TARGET_PER_ZONE 1U

typedef struct {
    uint16_t address;
} VL53L5CX_Platform;

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform);
uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);
uint8_t VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);

#endif
