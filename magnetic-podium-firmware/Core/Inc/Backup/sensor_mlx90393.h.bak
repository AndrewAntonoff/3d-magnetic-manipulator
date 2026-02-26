#ifndef SENSOR_MLX90393_H
#define SENSOR_MLX90393_H

#include "main.h"
#include "config.h"
#include <stdint.h>

typedef struct {
    // Аппаратная конфигурация
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    // Геометрия (для триангуляции)
    SensorGeometry_t geometry;

    // Измеренные данные
    float magnetic_field[3];   // X, Y, Z в µT
    float temperature;          // в °C
    uint32_t last_read_time;

    // Калибровочные параметры
    float offset[3];            // смещение по осям (µT)
    float scale[3];             // масштаб (пока не используется)
    uint8_t is_calibrated;

    // Состояние
    uint8_t is_connected;
    uint8_t read_error_count;
    uint8_t gain_sel;            // текущий GAIN_SEL (0..7)

    // Статистика
    uint32_t total_reads;
    uint32_t failed_reads;
    float average_read_time_ms;
} MLX90393_t;

// Глобальный массив датчиков (определён в .c)
extern MLX90393_t sensors[NUM_SENSORS];

// Прототипы функций
void Sensors_Init(void);
uint8_t Read_Sensor_With_Gain(uint8_t sensor_idx);
void Calibrate_Offset_Procedure(uint8_t sensor_idx);
uint8_t Read_Sensor_Calibrated(uint8_t sensor_idx);
void Save_Calibration_To_Flash(void);
void Load_Calibration_From_Flash(void);

// Вспомогательные (опционально)
uint8_t Test_Sensor_Connection(uint8_t sensor_idx);
float Calculate_Magnetic_Field_Strength(uint8_t sensor_idx);
uint8_t Quick_Read_Sensor(uint8_t sensor_idx);
uint8_t Read_Sensor(uint8_t sensor_idx);
void Calibrate_Sensors_Start(void);
void Get_Sensor_Stats_String(char *buffer, uint16_t buffer_size);

#endif
