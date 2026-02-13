#include "coil_driver.h"
#include "config.h" // <-- Добавить, чтобы получить PWM_MAX_VALUE, OVERCURRENT_THRESHOLD
#include "main.h" // Для HAL и GPIO
#include "tim.h" // Для HAL_TIM_PWM_Start/Stop, __HAL_TIM_SET_COMPARE
#include "gpio.h" // Для HAL_GPIO_WritePin
#include <string.h> // Для strcpy, strcmp, memset, strlen, strncat
#include <stdio.h>  // Для snprintf

// Массив катушек
Coil_t coils[NUM_COILS];
CoilTest_t coil_test = {0};

void Coils_Init(void) {
    // Инициализация всех катушек
    for(int i = 0; i < NUM_COILS; i++) {
        coils[i].current_pwm = 0.0f;
        coils[i].target_pwm = 0.0f;
        coils[i].is_faulty = 0;

        // DIR порт и пин (из main.h)
        coils[i].dir_port = GPIOE;  // Все катушки на GPIOE

        // Назначение dir_pin - исправленный порядок:
        switch(i) {
            case 0: coils[i].dir_pin = Coil1_DIR_Pin; break;  // PE0
            case 1: coils[i].dir_pin = Coil2_DIR_Pin; break;  // PE1
            case 2: coils[i].dir_pin = Coil3_DIR_Pin; break;  // PE2
            case 3: coils[i].dir_pin = Coil4_DIR_Pin; break;  // PE3
            case 4: coils[i].dir_pin = Coil5_DIR_Pin; break;  // PE4
            case 5: coils[i].dir_pin = Coil6_DIR_Pin; break;  // PE5
            case 6: coils[i].dir_pin = Coil7_DIR_Pin; break;  // PE6
            case 7: coils[i].dir_pin = Coil8_DIR_Pin; break;  // PE7
            case 8: coils[i].dir_pin = Coil9_DIR_Pin; break;  // PE8
            case 9: coils[i].dir_pin = Coil10_DIR_Pin; break; // PE10
            case 10: coils[i].dir_pin = Coil11_DIR_Pin; break; // PE12
            case 11: coils[i].dir_pin = Coil12_DIR_Pin; break; // PE15
            default: coils[i].dir_pin = GPIO_PIN_0;
        }

        // PWM timers и channels - ВАЖНО: соответствие CubeMX
        // TIM1: PE9 (CH1), PE11 (CH2), PE13 (CH3), PE14 (CH4)
        // TIM2: PA0 (CH1), PA1 (CH2), PA2 (CH3), PA3 (CH4)
        // TIM3: PA6 (CH1), PA7 (CH2), PB0 (CH3), PB1 (CH4)

        if (i < 4) {
            // Первые 4 катушки на TIM1
            coils[i].timer = &htim1;
            coils[i].channel = TIM_CHANNEL_1 + i;  // i=0 → CH1, i=1 → CH2, i=2 → CH3, i=3 → CH4
        } else if (i < 8) {
            // Следующие 4 катушки на TIM2
            coils[i].timer = &htim2;
            coils[i].channel = TIM_CHANNEL_1 + (i - 4);  // i=4 → CH1, i=5 → CH2, i=6 → CH3, i=7 → CH4
        } else {
            // Последние 4 катушки на TIM3
            coils[i].timer = &htim3;
            coils[i].channel = TIM_CHANNEL_1 + (i - 8);  // i=8 → CH1, i=9 → CH2, i=10 → CH3, i=11 → CH4
        }
    }
}

void Set_Coil_Power(uint8_t coil_idx, float power) {
    if(coil_idx >= NUM_COILS) return;

    // Ограничение мощности
    if(power > 1.0f) power = 1.0f;
    if(power < -1.0f) power = -1.0f;

    Coil_t *coil = &coils[coil_idx];

    // Установка направления
    if(power >= 0) {
        HAL_GPIO_WritePin(coil->dir_port, coil->dir_pin, GPIO_PIN_RESET);  // Positive
    } else {
        HAL_GPIO_WritePin(coil->dir_port, coil->dir_pin, GPIO_PIN_SET);    // Negative
        power = -power;  // Абсолютное для PWM
    }

    // Реальная установка PWM
    uint32_t compare = (uint32_t)(power * PWM_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(coil->timer, coil->channel, compare);

    coils[coil_idx].current_pwm = power;
    coils[coil_idx].target_pwm = power;

    // Лог
    Debug_Print(LOG_LEVEL_INFO, "Coil %d set to %.2f\r\n", coil_idx, power);
}

void Set_All_Coils_Power(float power) {
    for(int i = 0; i < NUM_COILS; i++) {
        Set_Coil_Power(i, power);
    }
}

void Stop_All_Coils(void) {
    Set_All_Coils_Power(0.0f);
}

float Get_Coil_Power(uint8_t coil_idx) {
    if(coil_idx >= NUM_COILS) return 0.0f;
    return coils[coil_idx].current_pwm;
}

void Start_Coil_Test(uint8_t coil_idx, float start_pwm, float end_pwm,
                     float step, uint32_t step_duration) {
    coil_test.coil_index = coil_idx;
    coil_test.test_pwm_start = start_pwm;
    coil_test.test_pwm_end = end_pwm;
    coil_test.test_pwm_step = step;
    coil_test.step_duration_ms = step_duration;
    coil_test.test_in_progress = 1;
    coil_test.test_start_time = HAL_GetTick();
    coil_test.steps_completed = 0;
}

void Stop_Coil_Test(void) {
    coil_test.test_in_progress = 0;
}

void Process_Coil_Test(void) {
    if(!coil_test.test_in_progress) return;

    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - coil_test.test_start_time;

    // Расчет текущего шага
    uint32_t current_step = elapsed / coil_test.step_duration_ms;

    if(current_step > coil_test.steps_completed) {
        float current_pwm = coil_test.test_pwm_start +
                           (coil_test.test_pwm_step * current_step);

        if((coil_test.test_pwm_step > 0 && current_pwm <= coil_test.test_pwm_end) ||
           (coil_test.test_pwm_step < 0 && current_pwm >= coil_test.test_pwm_end)) {
            Set_Coil_Power(coil_test.coil_index, current_pwm);
            coil_test.steps_completed = current_step;
            Debug_Print(LOG_LEVEL_INFO, "Test step %d: PWM %.2f\r\n", current_step, current_pwm);
        } else {
            Stop_Coil_Test();
            Set_Coil_Power(coil_test.coil_index, 0.0f);
        }
    }
}

uint8_t Is_Coil_Test_Running(void) {
    return coil_test.test_in_progress;
}

void Check_Coils_Safety(void) {
    // Проверка безопасности катушек
    for(int i = 0; i < NUM_COILS; i++) {
        // Проверка на overpower
        if(fabs(coils[i].current_pwm) > 0.8f) {
            if(!coils[i].is_faulty) {
                coils[i].is_faulty = 1;
                strcpy(coils[i].fault_reason, "Overpower");
            }
        } else {
            if(coils[i].is_faulty && strcmp(coils[i].fault_reason, "Overpower") == 0) {
                coils[i].is_faulty = 0;
            }
        }

        // Пример проверки overcurrent (реализуйте Read_ADC_Current если есть ADC)
        // coils[i].current_ma = Read_ADC_Current(i);  // Заглушка
        if(coils[i].current_ma > OVERCURRENT_THRESHOLD) {
            coils[i].is_faulty = 1;
            strcpy(coils[i].fault_reason, "Overcurrent");
            Set_Coil_Power(i, 0.0f);
            Debug_Print(LOG_LEVEL_ERROR, "Fault on coil %d: %s\r\n", i, coils[i].fault_reason);
        }
    }
}

void Reset_Coil_Fault(uint8_t coil_idx) {
    if(coil_idx < NUM_COILS) {
        coils[coil_idx].is_faulty = 0;
        memset(coils[coil_idx].fault_reason, 0, sizeof(coils[coil_idx].fault_reason));
    }
}

uint8_t Get_Coil_Fault_Status(uint8_t coil_idx) {
    if(coil_idx >= NUM_COILS) return 0;
    return coils[coil_idx].is_faulty;
}

const char* Get_Coil_Fault_Reason(uint8_t coil_idx) {
    if(coil_idx >= NUM_COILS) return "";
    return coils[coil_idx].fault_reason;
}

void Get_Coils_Status_String(char* buffer, uint16_t buffer_size) {
    snprintf(buffer, buffer_size, "Coils: ");
    for(int i = 0; i < NUM_COILS; i++) {
        char temp[16];
        snprintf(temp, sizeof(temp), "%.2f ", coils[i].current_pwm);
        strncat(buffer, temp, buffer_size - strlen(buffer) - 1);  // Безопасный strcat
    }
}

float Get_Coils_Average_Power(void) {
    float sum = 0.0f;
    for(int i = 0; i < NUM_COILS; i++) {
        sum += fabs(coils[i].current_pwm);
    }
    return sum / NUM_COILS;
}

float Get_Coils_Max_Power(void) {
    float max_power = 0.0f;
    for(int i = 0; i < NUM_COILS; i++) {
        float power = fabs(coils[i].current_pwm);
        if(power > max_power) {
            max_power = power;
        }
    }
    return max_power;
}

uint32_t Get_Coils_Total_On_Time(void) {
    uint32_t total = 0;
    for(int i = 0; i < NUM_COILS; i++) {
        total += coils[i].total_on_time_ms;
    }
    return total;
}
