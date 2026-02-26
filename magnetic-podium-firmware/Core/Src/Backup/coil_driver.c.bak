#include "coil_driver.h"
#include "config.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "debug_console.h"

// Массив катушек
static Coil_t coils[NUM_COILS];
static CoilTest_t coil_test = {0};

void Coils_Init(void)
{
    for (int i = 0; i < NUM_COILS; i++) {
        coils[i].current_pwm = 0.0f;
        coils[i].target_pwm = 0.0f;
        coils[i].enabled_time_ms = 0;
        coils[i].temperature_c = 25.0f;
        coils[i].current_ma = 0;
        coils[i].total_on_time_ms = 0;
        coils[i].activation_count = 0;
        coils[i].is_faulty = 0;
        memset(coils[i].fault_reason, 0, sizeof(coils[i].fault_reason));

        // Назначение DIR портов и пинов в соответствии с main.h
        switch (i) {
            case 0:  coils[i].dir_port = Coil1_DIR_GPIO_Port; coils[i].dir_pin = Coil1_DIR_Pin; break;
            case 1:  coils[i].dir_port = Coil2_DIR_GPIO_Port; coils[i].dir_pin = Coil2_DIR_Pin; break;
            case 2:  coils[i].dir_port = Coil3_DIR_GPIO_Port; coils[i].dir_pin = Coil3_DIR_Pin; break;
            case 3:  coils[i].dir_port = Coil4_DIR_GPIO_Port; coils[i].dir_pin = Coil4_DIR_Pin; break;
            case 4:  coils[i].dir_port = Coil5_DIR_GPIO_Port; coils[i].dir_pin = Coil5_DIR_Pin; break;
            case 5:  coils[i].dir_port = Coil6_DIR_GPIO_Port; coils[i].dir_pin = Coil6_DIR_Pin; break;
            case 6:  coils[i].dir_port = Coil7_DIR_GPIO_Port; coils[i].dir_pin = Coil7_DIR_Pin; break;
            case 7:  coils[i].dir_port = Coil8_DIR_GPIO_Port; coils[i].dir_pin = Coil8_DIR_Pin; break;
            case 8:  coils[i].dir_port = Coil9_DIR_GPIO_Port; coils[i].dir_pin = Coil9_DIR_Pin; break;
            case 9:  coils[i].dir_port = Coil10_DIR_GPIO_Port; coils[i].dir_pin = Coil10_DIR_Pin; break;
            case 10: coils[i].dir_port = Coil11_DIR_GPIO_Port; coils[i].dir_pin = Coil11_DIR_Pin; break;
            case 11: coils[i].dir_port = Coil12_DIR_GPIO_Port; coils[i].dir_pin = Coil12_DIR_Pin; break;
            default: coils[i].dir_port = GPIOE; coils[i].dir_pin = 0; break;
        }

        // Назначение таймеров и каналов в соответствии с .ioc
        if (i < 4) {
            // Первые 4 катушки на TIM1
            coils[i].timer = &htim1;
            coils[i].channel = (i == 0) ? TIM_CHANNEL_1 :
                               (i == 1) ? TIM_CHANNEL_2 :
                               (i == 2) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
        } else if (i < 8) {
            // Следующие 4 на TIM2
            coils[i].timer = &htim2;
            coils[i].channel = (i == 4) ? TIM_CHANNEL_1 :
                               (i == 5) ? TIM_CHANNEL_2 :
                               (i == 6) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
        } else {
            // Последние 4 на TIM3
            coils[i].timer = &htim3;
            coils[i].channel = (i == 8) ? TIM_CHANNEL_1 :
                               (i == 9) ? TIM_CHANNEL_2 :
                               (i == 10) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
        }
    }

    // Запуск ШИМ для всех каналов (можно перенести в main после инициализации)
    // Здесь только инициализация структур, запуск делаем отдельно.
}

void Set_Coil_Power(uint8_t coil_idx, float power)
{
	Debug_Print(LOG_LEVEL_INFO, "Set_Coil_Power(%d, %f)\r\n", coil_idx, power);
    if (coil_idx >= NUM_COILS) return;
    if (coils[coil_idx].is_faulty) return;

    // Ограничение мощности
    if (power > 1.0f) power = 1.0f;
    if (power < -1.0f) power = -1.0f;

    Coil_t *coil = &coils[coil_idx];

    // Установка направления
    if (power >= 0) {
        HAL_GPIO_WritePin(coil->dir_port, coil->dir_pin, GPIO_PIN_RESET); // положительное направление
    } else {
        HAL_GPIO_WritePin(coil->dir_port, coil->dir_pin, GPIO_PIN_SET);   // отрицательное
        power = -power;
    }

    // Установка ШИМ
    uint32_t compare = (uint32_t)(power * PWM_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(coil->timer, coil->channel, compare);

    coil->current_pwm = (power >= 0) ? power : -power; // сохраняем знак
    coil->target_pwm = power;
    coil->enabled_time_ms = HAL_GetTick();

    if (fabsf(power) > 0.01f) {
        coil->activation_count++;
    }
}

void Set_All_Coils_Power(float power)
{
    for (int i = 0; i < NUM_COILS; i++) {
        Set_Coil_Power(i, power);
    }
}

void Stop_All_Coils(void)
{
    Set_All_Coils_Power(0.0f);
}

float Get_Coil_Power(uint8_t coil_idx)
{
    if (coil_idx >= NUM_COILS) return 0.0f;
    return coils[coil_idx].current_pwm;
}

// --- Тестирование катушек ---
void Start_Coil_Test(uint8_t coil_idx, float start_pwm, float end_pwm,
                     float step, uint32_t step_duration)
{
    if (coil_idx >= NUM_COILS) return;
    coil_test.coil_index = coil_idx;
    coil_test.test_pwm_start = start_pwm;
    coil_test.test_pwm_end = end_pwm;
    coil_test.test_pwm_step = step;
    coil_test.step_duration_ms = step_duration;
    coil_test.test_in_progress = 1;
    coil_test.test_start_time = HAL_GetTick();
    coil_test.steps_completed = 0;
}

void Stop_Coil_Test(void)
{
    coil_test.test_in_progress = 0;
    Set_Coil_Power(coil_test.coil_index, 0.0f);
}

void Process_Coil_Test(void)
{
    if (!coil_test.test_in_progress) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - coil_test.test_start_time;
    uint32_t step = elapsed / coil_test.step_duration_ms;

    if (step > coil_test.steps_completed) {
        float power = coil_test.test_pwm_start +
                      coil_test.test_pwm_step * step;
        // Проверка окончания
        if ((coil_test.test_pwm_step > 0 && power <= coil_test.test_pwm_end) ||
            (coil_test.test_pwm_step < 0 && power >= coil_test.test_pwm_end)) {
            Set_Coil_Power(coil_test.coil_index, power);
            coil_test.steps_completed = step;
        } else {
            Stop_Coil_Test();
        }
    }
}

uint8_t Is_Coil_Test_Running(void)
{
    return coil_test.test_in_progress;
}

// --- Безопасность ---
void Check_Coils_Safety(void)
{
    for (int i = 0; i < NUM_COILS; i++) {
        // Пример: перегрузка по току (если есть АЦП)
        // if (coils[i].current_ma > OVERCURRENT_THRESHOLD) {
        //     coils[i].is_faulty = 1;
        //     strcpy(coils[i].fault_reason, "Overcurrent");
        //     Set_Coil_Power(i, 0.0f);
        // }
        // Пример: превышение температуры
        if (coils[i].temperature_c > 80.0f) {
            coils[i].is_faulty = 1;
            strcpy(coils[i].fault_reason, "Overtemp");
            Set_Coil_Power(i, 0.0f);
        }
    }
}

void Reset_Coil_Fault(uint8_t coil_idx)
{
    if (coil_idx < NUM_COILS) {
        coils[coil_idx].is_faulty = 0;
        memset(coils[coil_idx].fault_reason, 0, sizeof(coils[coil_idx].fault_reason));
    }
}

uint8_t Get_Coil_Fault_Status(uint8_t coil_idx)
{
    if (coil_idx >= NUM_COILS) return 0;
    return coils[coil_idx].is_faulty;
}

const char* Get_Coil_Fault_Reason(uint8_t coil_idx)
{
    if (coil_idx >= NUM_COILS) return "";
    return coils[coil_idx].fault_reason;
}

// --- Статус ---
void Get_Coils_Status_String(char* buffer, uint16_t buffer_size)
{
    int len = snprintf(buffer, buffer_size, "Coils: ");
    for (int i = 0; i < NUM_COILS; i++) {
        len += snprintf(buffer + len, buffer_size - len, "%d:%.2f%s ",
                        i, coils[i].current_pwm,
                        coils[i].is_faulty ? "(F)" : "");
    }
}

float Get_Coils_Average_Power(void)
{
    float sum = 0.0f;
    for (int i = 0; i < NUM_COILS; i++) {
        sum += fabsf(coils[i].current_pwm);
    }
    return sum / NUM_COILS;
}

float Get_Coils_Max_Power(void)
{
    float max = 0.0f;
    for (int i = 0; i < NUM_COILS; i++) {
        float p = fabsf(coils[i].current_pwm);
        if (p > max) max = p;
    }
    return max;
}

uint32_t Get_Coils_Total_On_Time(void)
{
    uint32_t total = 0;
    for (int i = 0; i < NUM_COILS; i++) {
        total += coils[i].total_on_time_ms;
    }
    return total;
}
