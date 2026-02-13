#ifndef __SENSOR_MLX90393_H
#define __SENSOR_MLX90393_H

#include "main.h"
#include "config.h"

// Геометрия датчиков (в мм)
typedef struct {
    float x, y, z;  // Позиция датчика относительно центра чаши
    float orientation[3];  // Ориентация датчика (вектор нормали)
} SensorGeometry_t;

// Структура датчика MLX90393
typedef struct {
    // Аппаратная конфигурация
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    // Геометрия
    SensorGeometry_t geometry;

    // Данные
    float magnetic_field[3];    // X, Y, Z в мТл
    float temperature;          // Температура в °C
    uint32_t last_read_time;

    // Калибровка
    float offset[3];
    float scale[3];
    float rotation_matrix[3][3];

    // Состояние
    uint8_t is_connected;
    uint8_t is_calibrated;
    uint8_t read_error_count;
    uint8_t config_registers[8];
    uint8_t gain_sel;

    // Статистика
    uint32_t total_reads;
    uint32_t failed_reads;
    float average_read_time_ms;
} MLX90393_t;

// Структура для 3D позиции
typedef struct {
    float x;  // мм
    float y;  // мм
    float z;  // мм
    float confidence;  // 0.0 - 1.0
    uint32_t timestamp;
} Position3D_t;

// Прототипы функций
void Sensors_Init(void);
void Sensors_Deinit(void);
uint8_t Read_Sensor(uint8_t sensor_idx);
uint8_t Read_All_Sensors(void);

// Калибровка
void Calibrate_Sensors_Start(void);
void Calibrate_Sensors_Stop(void);
uint8_t Is_Calibration_Running(void);
void Save_Calibration_To_Flash(void);
void Load_Calibration_From_Flash(void);

// Обработка данных
void Calculate_Ball_Position(Position3D_t* position);
void Apply_Calibration(uint8_t sensor_idx);
float Calculate_Magnetic_Field_Strength(uint8_t sensor_idx);

// Диагностика
uint8_t Test_Sensor_Connection(uint8_t sensor_idx);
void Run_Sensor_Self_Test(uint8_t sensor_idx);
uint8_t Get_Sensor_Health_Status(uint8_t sensor_idx);
uint8_t Quick_Read_Sensor(uint8_t sensor_idx);

// Мониторинг
void Get_Sensor_Data_String(char* buffer, uint16_t buffer_size);
void Get_Sensor_Stats_String(char* buffer, uint16_t buffer_size);
float Get_Sensor_Read_Frequency(uint8_t sensor_idx);

// Геометрия
void Initialize_Sensor_Geometry(void);
const SensorGeometry_t* Get_Sensor_Geometry(uint8_t sensor_idx);

// Магнитное поле
void Calculate_Magnetic_Field_At_Point(float x, float y, float z, float* Bx, float* By, float* Bz);

// SPI тестовые функции
void Test_SPI_Bus(uint8_t test_pattern);
void Measure_SPI_Timing(void);
uint8_t Verify_SPI_Communication(void);

// Расширенная диагностика
uint8_t MLX90393_Read_Register(uint8_t sensor_idx, uint8_t addr, uint16_t *value);
void MLX90393_Reset_Sensor(uint8_t sensor_idx);
uint8_t MLX90393_Get_Raw_Data(uint8_t sensor_idx, int16_t *t, int16_t *x, int16_t *y, int16_t *z);

#endif /* __SENSOR_MLX90393_H */
