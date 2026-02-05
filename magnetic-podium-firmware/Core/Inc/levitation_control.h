#ifndef __LEVITATION_CONTROL_H
#define __LEVITATION_CONTROL_H

#include "main.h"
#include "config.h"
#include "coil_driver.h"
#include "sensor_mlx90393.h"

// Структура геометрии катушки
typedef struct {
    float x, y, z;  // Позиция катушки (мм)
    float orientation[3];  // Ориентация катушки (нормализованный вектор)
} CoilGeometry_t;

// Внешние объявления
extern SystemState_t system_state;
extern OperationMode_t current_mode;
extern MLX90393_t sensors[NUM_SENSORS];

// Прототипы функций
void Initialize_Coil_Geometry(void);
void Initialize_Sensor_Geometry(void);
void Calculate_Ball_Position(Position3D_t* position);
void Update_PID_Controller(float dt);
void Calculate_Coil_Forces(float fx, float fy, float fz, float* coil_powers);
void Apply_Levitation_Control(void);
void Start_Levitation(void);
void Stop_Levitation(void);
void Set_Levitation_Target(float x, float y, float z);
void Get_Levitation_Status(char* buffer, uint16_t buffer_size);
void Test_All_Hardware(void);
void Calibrate_Sensors_Offset(void);

#endif /* __LEVITATION_CONTROL_H */
