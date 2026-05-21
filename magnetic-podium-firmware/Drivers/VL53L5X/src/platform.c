#include "platform.h"
#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
    uint8_t status = HAL_I2C_Mem_Read(&hi2c1, p_platform->address, RegisterAdress, I2C_MEMADD_SIZE_16BIT, p_value, 1, 100);
    return (status == HAL_OK) ? 0 : 255;
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
    uint8_t status = HAL_I2C_Mem_Write(&hi2c1, p_platform->address, RegisterAdress, I2C_MEMADD_SIZE_16BIT, &value, 1, 100);
    return (status == HAL_OK) ? 0 : 255;
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    // VL53L5X can demand quite large multi-reads. Ensure timeout is adequate.
    uint8_t status = HAL_I2C_Mem_Read(&hi2c1, p_platform->address, RegisterAdress, I2C_MEMADD_SIZE_16BIT, p_values, size, 1000);
    return (status == HAL_OK) ? 0 : 255;
}

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    // Write firmware uses WrMulti frequently with large chunks.
    uint8_t status = HAL_I2C_Mem_Write(&hi2c1, p_platform->address, RegisterAdress, I2C_MEMADD_SIZE_16BIT, p_values, size, 2000);
    return (status == HAL_OK) ? 0 : 255;
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform) {
    (void)p_platform;
    // For a soft reset via I2C, usually VL53L5X handles it inside vl53l5cx_init.
    // If you have connected the LPn (Low Power) pin or I2C_RST pin,
    // you would toggle it LOW then HIGH here with HAL_GPIO_WritePin.
    return 0; 
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs) {
    (void)p_platform;
    HAL_Delay(TimeMs);
    return 0;
}

uint8_t VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint32_t i, tmp;
    for(i = 0; i < size; i = i + 4) {
        tmp = (
          ((uint32_t)buffer[i]<<24)
        | ((uint32_t)buffer[i+1]<<16)
        | ((uint32_t)buffer[i+2]<<8)
        | ((uint32_t)buffer[i+3])
        );
        memcpy(&(buffer[i]), &tmp, 4);
    }
    return 0;
}
