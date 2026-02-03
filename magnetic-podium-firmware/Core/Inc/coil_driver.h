#ifndef __COIL_DRIVER_H
#define __COIL_DRIVER_H

#include "main.h"
#include "config.h"

// Структура катушки
typedef struct {
    // Аппаратная конфигурация
    TIM_HandleTypeDef* timer;
    uint32_t channel;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;

    // Состояние
    float current_pwm;           // -1.0 до +1.0
    float target_pwm;
    uint32_t enabled_time_ms;
    float temperature_c;         // Температура (если есть датчик)
    uint32_t current_ma;         // Ток (если есть измерение)

    // Статистика
    uint32_t total_on_time_ms;
    uint32_t activation_count;
    uint8_t is_faulty;
    char fault_reason[32];
} Coil_t;

// Структура теста катушки
typedef struct {
    uint8_t coil_index;
    float test_pwm_start;
    float test_pwm_end;
    float test_pwm_step;
    uint32_t step_duration_ms;
    uint8_t test_in_progress;
    uint32_t test_start_time;
    uint32_t steps_completed;
} CoilTest_t;

// Прототипы функций
void Coils_Init(void);
void Set_Coil_Power(uint8_t coil_idx, float power);
void Set_All_Coils_Power(float power);
void Stop_All_Coils(void);
float Get_Coil_Power(uint8_t coil_idx);

// Тестирование
void Start_Coil_Test(uint8_t coil_idx, float start_pwm, float end_pwm,
                     float step, uint32_t step_duration);
void Stop_Coil_Test(void);
void Process_Coil_Test(void);
uint8_t Is_Coil_Test_Running(void);

// Безопасность
void Check_Coils_Safety(void);
void Reset_Coil_Fault(uint8_t coil_idx);
uint8_t Get_Coil_Fault_Status(uint8_t coil_idx);
const char* Get_Coil_Fault_Reason(uint8_t coil_idx);

// Мониторинг
void Get_Coils_Status_String(char* buffer, uint16_t buffer_size);
float Get_Coils_Average_Power(void);
float Get_Coils_Max_Power(void);
uint32_t Get_Coils_Total_On_Time(void);

#endif /* __COIL_DRIVER_H */
