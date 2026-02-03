/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Your Company.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "coil_driver.h"
#include "sensor_mlx90393.h"
#include "debug_console.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Глобальные структуры
SystemState_t system_state = {
    .coils_enabled = 0,
    .sensors_enabled = 0,
    .monitoring_active = 0,
    .calibration_done = 0,
    .system_uptime_ms = 0,
    .cpu_usage_percent = 0.0f
};

OperationMode_t current_mode = MODE_IDLE;

// Буферы
char console_buffer[512];
char monitor_buffer[1024];
uint8_t command_buffer[128];
uint16_t command_index = 0;
uint8_t RxChar;  // Глобальная или static в main

// Тайминги
uint32_t last_monitor_time = 0;
uint32_t monitor_interval = 100; // мс
uint32_t last_cpu_measure = 0;
uint32_t idle_counter_start = 0;
uint32_t idle_counter = 0;

// Статистика
uint32_t total_commands = 0;
uint32_t bytes_received = 0;
uint32_t bytes_transmitted = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Команды консоли
#define CMD_HELP          "help"
#define CMD_STATUS        "status"
#define CMD_TEST_COIL     "testcoil"
#define CMD_TEST_ALL      "testall"
#define CMD_TEST_SPI      "testspi"
#define CMD_TEST_SENSOR   "testsensor"
#define CMD_CALIBRATE     "calibrate"
#define CMD_MONITOR       "monitor"
#define CMD_STOP          "stop"
#define CMD_RESET         "reset"
#define CMD_COIL          "coil"
#define CMD_SENSOR        "sensor"
#define CMD_SPI           "spi"
#define CMD_EXPORT        "export"
#define CMD_CONFIG        "config"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Флаги
uint8_t new_command = 0;
uint8_t system_ready = 0;
uint8_t emergency_stop = 0;

// Глобальные смещения для датчика 0
float offset_x = 0, offset_y = 0, offset_z = 0;
uint8_t offset_calibrated = 0;

// Тестовые переменные
float test_coil_power = 0.0f;
uint8_t test_coil_index = 0;
uint8_t test_sensor_index = 0;
uint8_t test_spi_pattern = 0xAA;
// Объявление массива датчиков (определен в sensor_mlx90393.c)
extern MLX90393_t sensors[NUM_SENSORS];

uint8_t streaming_active = 0;        // Флаг активной передачи
uint32_t stream_interval_ms = 50;    // Интервал передачи (20 Гц)
uint32_t last_stream_time = 0;       // Время последней передачи

// ПИД параметры (заглушка)
typedef struct {
    float Kp, Ki, Kd;
    float setpoint[3];
    float output[3];
} PID_Controller_t;

PID_Controller_t pid_controller = {
    .Kp = 1.0f,
    .Ki = 0.1f,
    .Kd = 0.01f,
    .setpoint = {0, 0, 0},
    .output = {0, 0, 0}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MPU_Config(void);
/* USER CODE BEGIN PFP */
// Прототипы внутренних функций
void System_Init(void);
void System_Deinit(void);
void Process_Console_Commands(void);
void Update_System_Status(void);
void Handle_Emergency_Stop(void);
float Calculate_Temperature(int16_t t_raw);

// Функции тестирования
void Run_Full_System_Test(void);
void Run_Coil_Test_Suite(void);
void Run_Sensor_Test_Suite(void);
void Run_SPI_Test_Suite(void);
void Run_Integration_Test(void);

// Вспомогательные функции
void Calculate_CPU_Usage(void);
void Update_Uptime(void);
void Save_System_Config(void);
void Load_System_Config(void);

// Прототипы функций, которые определены ниже в файле
void Show_Help_Menu(void);
void Show_System_Status(void);
void Test_SPI_Pins(void);
void Test_SPI_Communication(void);
void Test_Single_Sensor(void);
uint8_t Quick_Read_Sensor(uint8_t sensor_idx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Инициализация системы
void System_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Инициализация отладки
    Debug_Init(&huart2);
    Debug_Print(LOG_LEVEL_INFO, "=== Magnetic Manipulator System Initialization ===\r\n");

    // Инициализация пина CS для датчика
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Настраиваем PC0 (CS для датчика 1) как выход
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Устанавливаем высокий уровень (CS неактивен)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

    // Инициализация катушек
    Debug_Print(LOG_LEVEL_INFO, "Initializing coil drivers...\r\n");
    Coils_Init();
    system_state.coils_enabled = 1;

    // Инициализация датчиков
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");
    Sensors_Init();
    system_state.sensors_enabled = 1;

    // Загрузка конфигурации
    Load_System_Config();

    // Запуск PWM для всех таймеров
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void System_Deinit(void) {
    // Деинициализация
}

void Update_System_Status(void) {
    Update_Uptime();
    Calculate_CPU_Usage();
    Check_Coils_Safety();
    // Добавьте другие проверки статуса
}

void Handle_Emergency_Stop(void) {
    Stop_All_Coils();
    Debug_Print(LOG_LEVEL_ERROR, "Emergency stop activated!\r\n");
    emergency_stop = 0;
    current_mode = MODE_FAULT;
}

void Test_Single_Sensor(void) {
    Debug_Print(LOG_LEVEL_INFO, "=== Testing single sensor (idx=0) ===\r\n");

    // Простое чтение без сложной конфигурации
    uint8_t tx[2] = {0x00, 0x00};  // NOP command
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(GPIOC, Sensor1_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);

    HAL_GPIO_WritePin(GPIOC, Sensor1_CS_Pin, GPIO_PIN_SET);

    if(status == HAL_OK) {
        Debug_Print(LOG_LEVEL_INFO, "SPI OK: RX=[0x%02X 0x%02X]\r\n", rx[0], rx[1]);

        // Если получен не 0xFF, значит датчик отвечает
        if(rx[0] != 0xFF) {
            Debug_Print(LOG_LEVEL_INFO, "Sensor 0 detected!\r\n");
        } else {
            Debug_Print(LOG_LEVEL_WARNING, "Sensor 0 not responding (all 0xFF)\r\n");
        }
    } else {
        Debug_Print(LOG_LEVEL_ERROR, "SPI error: %d\r\n", status);
    }
}

uint8_t Quick_Read_Sensor(uint8_t sensor_idx) {
    Debug_Print(LOG_LEVEL_INFO, "Quick_Read_Sensor: Testing sensor %d\n", sensor_idx);

    if(sensor_idx >= NUM_SENSORS) {
        Debug_Print(LOG_LEVEL_ERROR, "Sensor index %d out of range\n", sensor_idx);
        return 0;
    }

    MLX90393_t *sensor = &sensors[sensor_idx];

    if (!sensor->is_connected) {
        Debug_Print(LOG_LEVEL_INFO, "Sensor %d marked as not connected, but trying anyway...\n", sensor_idx);
        // Попробуем все равно прочитать
    }
    // CS low
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    // 1. Reset для надежности
    uint8_t reset_cmd[2] = {0xF0, 0x00};
    uint8_t reset_resp[2] = {0};
    HAL_SPI_TransmitReceive(sensor->spi, reset_cmd, reset_resp, 2, 100);
    HAL_Delay(20);

    // 2. Exit any mode
    uint8_t exit_cmd[2] = {0x80, 0x00};
    uint8_t exit_resp[2] = {0};
    HAL_SPI_TransmitReceive(sensor->spi, exit_cmd, exit_resp, 2, 100);
    HAL_Delay(20);

    // 3. Start Measurement XYZT
    uint8_t start_cmd[2] = {0x3F, 0x00};  // ПРАВИЛЬНАЯ КОМАНДА: XYZ with temp
    uint8_t start_resp[2] = {0};
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(sensor->spi, start_cmd, start_resp, 2, 100);

    if (status != HAL_OK) {
        Debug_Print(LOG_LEVEL_ERROR, "SPI error during start: %d\n", status);
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        return 0;
    }

    Debug_Print(LOG_LEVEL_INFO, "Start response: 0x%02X 0x%02X\n", start_resp[0], start_resp[1]);

    // 4. Ждем измерение (минимум 1.2 мс для OSR=2)
    HAL_Delay(20);

    // 5. Read Measurement - 10 байт (статус + 8 байт данных)
    uint8_t read_cmd[10] = {0x4F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t read_resp[10] = {0};
    status = HAL_SPI_TransmitReceive(sensor->spi, read_cmd, read_resp, 10, 100);

    // CS high
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        Debug_Print(LOG_LEVEL_ERROR, "SPI error during start: %d\n", status);
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        return 0;

    }

    Debug_Print(LOG_LEVEL_INFO, "Read response: ");
    for(int i = 0; i < 10; i++) {
        Debug_Print(LOG_LEVEL_INFO, "0x%02X ", read_resp[i]);
    }
    Debug_Print(LOG_LEVEL_INFO, "\n");

    // Проверяем ERROR bit (бит 4)
    if ((read_resp[0] & 0x10) != 0) {
        Debug_Print(LOG_LEVEL_WARNING, "Sensor %d ERROR bit set (status=0x%02X)\n",
                   sensor_idx, read_resp[0]);
        return 0;
    }

    // Парсим данные (T, X, Y, Z - по 2 байта каждый)
    int16_t t_raw = (read_resp[2] << 8) | read_resp[3];
    int16_t x_raw = (read_resp[4] << 8) | read_resp[5];
    int16_t y_raw = (read_resp[6] << 8) | read_resp[7];
    int16_t z_raw = (read_resp[8] << 8) | read_resp[9];

    // Конвертация (примерные коэффициенты для gain=5, OSR=2)
    float lsb_xy = 0.751f;  // µT/LSB для X, Y
    float lsb_z = 1.210f;   // µT/LSB для Z

    sensor->magnetic_field[0] = x_raw * lsb_xy;
    sensor->magnetic_field[1] = y_raw * lsb_xy;
    sensor->magnetic_field[2] = z_raw * lsb_z;

    // Температура (формула из даташита)
    sensor->temperature = Calculate_Temperature(t_raw);

    Debug_Print(LOG_LEVEL_INFO, "Sensor %d: X=%.1f, Y=%.1f, Z=%.1f uT, T=%.1f C\n",
               sensor_idx,
               sensor->magnetic_field[0],
               sensor->magnetic_field[1],
               sensor->magnetic_field[2],
               sensor->temperature);
    Debug_Print(LOG_LEVEL_INFO, "Quick_Read_Sensor: Done\n");
    return 1;
}
void Debug_Sensor_Completely(uint8_t sensor_idx) {
    MLX90393_Reset_Sensor(sensor_idx);
}


void Run_Full_System_Test(void) {
    Debug_Print(LOG_LEVEL_INFO, "Running full system test...\r\n");
    Run_Coil_Test_Suite();
    Run_Sensor_Test_Suite();
    Run_SPI_Test_Suite();
    Run_Integration_Test();
    Debug_Print(LOG_LEVEL_INFO, "Full system test completed.\r\n");
}

void Run_Coil_Test_Suite(void) {
    Debug_Print(LOG_LEVEL_INFO, "Running coil test suite...\r\n");
    for(int i = 0; i < NUM_COILS; i++) {
        if(Get_Coil_Fault_Status(i)) {
            Debug_Print(LOG_LEVEL_WARNING, "Coil %d is faulty: %s\r\n", i, Get_Coil_Fault_Reason(i));
            continue;
        }
        Start_Coil_Test(i, 0.0f, 0.5f, 0.1f, 1000);  // Тест от 0 до 50%
        while(Is_Coil_Test_Running()) {
            Process_Coil_Test();
            HAL_Delay(100);
        }
        Debug_Print(LOG_LEVEL_INFO, "Coil %d test completed.\r\n", i);
    }
}

void Run_Sensor_Test_Suite(void) {
    Debug_Print(LOG_LEVEL_INFO, "Running sensor test suite...\r\n");
    for(int i = 0; i < NUM_SENSORS; i++) {
        if(Test_Sensor_Connection(i)) {
            Run_Sensor_Self_Test(i);
            Read_Sensor(i);
            if(Get_Sensor_Health_Status(i)) {
                Debug_Print(LOG_LEVEL_INFO, "Sensor %d is healthy.\r\n", i);
            } else {
                Debug_Print(LOG_LEVEL_WARNING, "Sensor %d health issue.\r\n", i);
            }
        } else {
            Debug_Print(LOG_LEVEL_ERROR, "Sensor %d not connected.\r\n", i);
        }
    }
}

void Run_SPI_Test_Suite(void) {
    Debug_Print(LOG_LEVEL_INFO, "Running SPI test suite...\r\n");
    Test_SPI_Bus(0xAA);
    Measure_SPI_Timing();
    if(Verify_SPI_Communication()) {
        Debug_Print(LOG_LEVEL_INFO, "SPI communication verified.\r\n");
    } else {
        Debug_Print(LOG_LEVEL_ERROR, "SPI communication failed.\r\n");
    }
}

void Run_Integration_Test(void) {
    Debug_Print(LOG_LEVEL_INFO, "Running integration test...\r\n");
    // Пример: Включить катушку и прочитать сенсор
    Set_Coil_Power(0, 0.3f);
    HAL_Delay(500);
    Read_All_Sensors();
    Stop_All_Coils();
    Debug_Print(LOG_LEVEL_INFO, "Integration test completed.\r\n");
}

void Calculate_CPU_Usage(void) {
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - last_cpu_measure;
    if(elapsed >= 1000) {  // Каждую секунду
        // Пример расчёта (адаптируйте под ваш код)
        float total_cycles = (SystemCoreClock / 1000) * elapsed;  // Пример
        float usage = 100.0f - ((idle_counter * 100.0f) / total_cycles);
        system_state.cpu_usage_percent = usage;
        Debug_Print(LOG_LEVEL_DEBUG, "CPU usage: %.1f%%\r\n", usage);
        idle_counter = 0;
        last_cpu_measure = current_time;
    }
}

void Update_Uptime(void) {
    system_state.system_uptime_ms = HAL_GetTick();
}

void Save_System_Config(void) {
    Debug_Print(LOG_LEVEL_INFO, "Saving system config to flash...\r\n");
    // Реализация: Используйте HAL_FLASH для записи параметров
}

void Load_System_Config(void) {
    Debug_Print(LOG_LEVEL_INFO, "Loading system config from flash...\r\n");
    // Реализация: Чтение из флеш
    Load_Calibration_From_Flash();
}

void Show_Help_Menu(void) {
	Debug_Print(LOG_LEVEL_INFO, "Available commands:\r\n");
	Debug_Print(LOG_LEVEL_INFO, "help - Show this menu\r\n");
	Debug_Print(LOG_LEVEL_INFO, "status - Show system status\r\n");
	Debug_Print(LOG_LEVEL_INFO, "check_spi - Check SPI configuration\r\n");
	Debug_Print(LOG_LEVEL_INFO, "sensor_test - Simple sensor test\r\n");
	Debug_Print(LOG_LEVEL_INFO, "Available commands:\r\n");
    Debug_Print(LOG_LEVEL_INFO, "help - Show this menu\r\n");
    Debug_Print(LOG_LEVEL_INFO, "status - Show system status\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testcoil <idx> <start_pwm> <end_pwm> <step> <duration_ms> - Test coil\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testall - Run full system test\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testspi - Run SPI test\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testsensor <idx> - Test sensor\r\n");
    Debug_Print(LOG_LEVEL_INFO, "calibrate - Start calibration\r\n");
    Debug_Print(LOG_LEVEL_INFO, "monitor - Start monitoring\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stop - Stop all coils\r\n");
    Debug_Print(LOG_LEVEL_INFO, "reset - Reset faults\r\n");
    Debug_Print(LOG_LEVEL_INFO, "coil <idx> <power> - Set coil power\r\n");
    Debug_Print(LOG_LEVEL_INFO, "sensor <idx> - Read sensor\r\n");
    Debug_Print(LOG_LEVEL_INFO, "spi - SPI commands\r\n");
    Debug_Print(LOG_LEVEL_INFO, "export - Export data\r\n");
    Debug_Print(LOG_LEVEL_INFO, "config - Config commands\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testpins - Test SPI pins (CS and SCK)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testallcoils - Test all coils sequentially\r\n");
    Debug_Print(LOG_LEVEL_INFO, "quicktest - Quick test all sensors\r\n");
    Debug_Print(LOG_LEVEL_INFO, "spiconfig - Show SPI configuration\r\n");
    Debug_Print(LOG_LEVEL_INFO, "parse - Parse last sensor data\r\n");
    Debug_Print(LOG_LEVEL_INFO, "simpletest - Simple sensor test with minimal config\r\n");
    Debug_Print(LOG_LEVEL_INFO, "debugsensor - Debug sensor with multiple readings\r\n");
    Debug_Print(LOG_LEVEL_INFO, "read0 - Force read sensor 0\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testcoeff - Test different sensor coefficients\r\n");
    Debug_Print(LOG_LEVEL_INFO, "checkmode - Check sensor communication mode\r\n");
    Debug_Print(LOG_LEVEL_INFO, "read0_correct - Read sensor with correct GAIN_SEL=0 coeff\r\n");
    Debug_Print(LOG_LEVEL_INFO, "testmagnet - Test sensor with magnet\r\n");
    Debug_Print(LOG_LEVEL_INFO, "calibrate_offset - Calibrate sensor offset\r\n");
    Debug_Print(LOG_LEVEL_INFO, "read_compensated - Read with offset compensation\r\n");
    Debug_Print(LOG_LEVEL_INFO, "coil_influence - Test coil influence on sensor\r\n");
    Debug_Print(LOG_LEVEL_INFO, "start_stream [interval_ms] - Start data streaming (default: 50 ms)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stop_stream - Stop data streaming\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stream_fast - Fast streaming (100 Hz)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stream_slow - Slow streaming (10 Hz)\r\n");
}

void Show_System_Status(void) {
    Debug_Print(LOG_LEVEL_INFO, "System status:\r\n");
    Debug_Print(LOG_LEVEL_INFO, "Mode: %d\r\n", current_mode);
    Debug_Print(LOG_LEVEL_INFO, "Uptime: %lu ms\r\n", system_state.system_uptime_ms);
    Debug_Print(LOG_LEVEL_INFO, "CPU usage: %.1f%%\r\n", system_state.cpu_usage_percent);
    Debug_Print(LOG_LEVEL_INFO, "Coils enabled: %d\r\n", system_state.coils_enabled);
    Debug_Print(LOG_LEVEL_INFO, "Sensors enabled: %d\r\n", system_state.sensors_enabled);
    Debug_Print(LOG_LEVEL_INFO, "Calibration done: %d\r\n", system_state.calibration_done);
    Debug_Print(LOG_LEVEL_INFO, "Monitoring active: %d\r\n", system_state.monitoring_active);
    Get_Coils_Status_String(console_buffer, sizeof(console_buffer));
    Debug_Print(LOG_LEVEL_INFO, "%s\r\n", console_buffer);
    Get_Sensor_Stats_String(console_buffer, sizeof(console_buffer));
    Debug_Print(LOG_LEVEL_INFO, "%s\r\n", console_buffer);
}

void Test_SPI_Pins(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    Debug_Print(LOG_LEVEL_INFO, "Testing SPI pins...\r\n");
    HAL_GPIO_WritePin(GPIOC, Sensor1_CS_Pin, GPIO_PIN_RESET);
    Debug_Print(LOG_LEVEL_INFO, "CS state: %d", HAL_GPIO_ReadPin(GPIOC, Sensor1_CS_Pin));

    // Тест CS (PC0) - должен переключаться
    Debug_Print(LOG_LEVEL_INFO, "Testing CS pin (PC0)...\r\n");
    for(int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(100);
    }

    // Тест SCK (PA5) - временно настраиваем как выход
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Debug_Print(LOG_LEVEL_INFO, "Testing SCK pin (PA5)...\r\n");
    for(int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay(50);
    }

    // Возвращаем SCK в режим SPI
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Debug_Print(LOG_LEVEL_INFO, "SPI pins test complete\r\n");
}

void Test_SPI_Communication(void) {
    Debug_Print(LOG_LEVEL_INFO, "=== SPI COMMUNICATION TEST ===\r\n");

    uint8_t tx_data[] = {0xAA, 0x55, 0x00, 0xFF};
    uint8_t rx_data[4] = {0};

    // Активируем CS
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(1);

    // Отправляем тестовые данные
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 4, 100);

    // Деактивируем CS
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

    if(status == HAL_OK) {
        Debug_Print(LOG_LEVEL_INFO, "SPI communication OK\r\n");
        Debug_Print(LOG_LEVEL_INFO, "Sent: ");
        for(int i = 0; i < 4; i++) {
            Debug_Print(LOG_LEVEL_INFO, "0x%02X ", tx_data[i]);
        }
        Debug_Print(LOG_LEVEL_INFO, "\r\n");
        Debug_Print(LOG_LEVEL_INFO, "Received: ");
        for(int i = 0; i < 4; i++) {
            Debug_Print(LOG_LEVEL_INFO, "0x%02X ", rx_data[i]);
        }
        Debug_Print(LOG_LEVEL_INFO, "\r\n");
    } else {
        Debug_Print(LOG_LEVEL_ERROR, "SPI communication FAILED: %d\r\n", status);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Обработка RxChar
        if (RxChar == '\r' || RxChar == '\n') {
            if (command_index > 0) {
                new_command = 1;
            }
        } else if (RxChar == '\b' || RxChar == 127) {
            if (command_index > 0) {
                command_index--;
            }
        } else if (command_index < sizeof(command_buffer) - 1) {
            command_buffer[command_index++] = RxChar;
            bytes_received++;
        }

        // Эхо
        HAL_UART_Transmit(&huart2, &RxChar, 1, 10);

        // Перезапустить прием
        HAL_UART_Receive_IT(&huart2, &RxChar, 1);
    }
}
float Calculate_Temperature(int16_t t_raw) {
    // Формула из даташита
    return ((float)t_raw - 46244.0f) / 45.2f + 25.0f;
}
uint8_t Fast_Read_Sensor(uint8_t sensor_idx, float *x, float *y, float *z) {
    if(sensor_idx >= NUM_SENSORS) return 0;

    MLX90393_t *sensor = &sensors[sensor_idx];
    if(!sensor->is_connected) return 0;

    // 1. Start measurement
    uint8_t start_cmd[2] = {0x3F, 0x00};
    uint8_t start_resp[2] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(sensor->spi, start_cmd, start_resp, 2, 100);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    // Wait for measurement (уменьшим до минимума)
    HAL_Delay(5);

    // 2. Read measurement
    uint8_t read_cmd[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
    uint8_t read_resp[10] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(sensor->spi, read_cmd, read_resp, 10, 100);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    // Parse data
    int16_t x_raw = (read_resp[4] << 8) | read_resp[5];
    int16_t y_raw = (read_resp[6] << 8) | read_resp[7];
    int16_t z_raw = (read_resp[8] << 8) | read_resp[9];

    // Convert with GAIN_SEL=0 coefficients
    const float lsb_xy = 0.751f;
    const float lsb_z = 1.210f;

    *x = x_raw * lsb_xy;
    *y = y_raw * lsb_xy;
    *z = z_raw * lsb_z;

    // Apply calibration
    if(sensor->is_calibrated) {
        *x -= sensor->offset[0];
        *y -= sensor->offset[1];
        *z -= sensor->offset[2];
    }

    return 1;
}
void Stream_Sensor_Data(void) {
    if(!streaming_active) return;

    uint32_t current_time = HAL_GetTick();
    if(current_time - last_stream_time < stream_interval_ms) {
        return;
    }
    last_stream_time = current_time;

    float x, y, z;
    if(Fast_Read_Sensor(0, &x, &y, &z)) {
        // Формат: X,Y,Z\n
        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "%.1f,%.1f,%.1f\n", x, y, z);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 10);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_Init();
  /* USER CODE END 1 */

  /* MPU Configuration*/

  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Инициализация системы
  System_Init();
  // Включаем прием по UART
  HAL_UART_Receive_IT(&huart2, &RxChar, 1);

  // Запускаем системный таймер
  last_monitor_time = HAL_GetTick();
  last_cpu_measure = HAL_GetTick();

  // Выводим приветствие
  Debug_Print(LOG_LEVEL_INFO, "\r\n");
  Debug_Print(LOG_LEVEL_INFO, "========================================\r\n");
  Debug_Print(LOG_LEVEL_INFO, "   MAGNETIC MANIPULATOR CONTROL SYSTEM\r\n");
  Debug_Print(LOG_LEVEL_INFO, "   Version: 2.0 | Built: %s %s\r\n", __DATE__, __TIME__);
  Debug_Print(LOG_LEVEL_INFO, "   Coils: %d | Sensors: %d\r\n", NUM_COILS, NUM_SENSORS);
  Debug_Print(LOG_LEVEL_INFO, "========================================\r\n\r\n");

  // Показываем помощь
  Show_Help_Menu();

  // Тест пинов SPI
  Test_SPI_Pins();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Обработка команд
	      Process_Console_Commands();

	      // Потоковая передача данных
	      Stream_Sensor_Data();

	      // Обновление статуса системы
	      Update_System_Status();

	      // Обработка тестов катушек
	      if(Is_Coil_Test_Running()) {
	          Process_Coil_Test();
	      }

	      // Обработка калибровки
	      if(Is_Calibration_Running()) {
	          // Калибровка в процессе
	      }

	      // Проверка аварийной остановки
	      if(emergency_stop) {
	          Handle_Emergency_Stop();
	      }

	      // Небольшая задержка для стабильности
	      if(!streaming_active) {
	          HAL_Delay(1);
	      }
	      // При активной потоковой передаче уменьшаем задержку
	      else {
	          HAL_Delay(0);
	      }
	  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // ВАЖНО: HSE вместо HSI
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;  // Включить HSE
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;  // PLL от HSE
    RCC_OscInitStruct.PLL.PLLM = 2;
    RCC_OscInitStruct.PLL.PLLN = 64;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
// Обработчики ошибок
void Error_Handler(void) {
    Debug_Print(LOG_LEVEL_ERROR, "Fatal error occurred! System halted.\r\n");

    // Мигаем светодиодом
    while(1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}

void Process_Console_Commands(void) {
    if (!new_command) return;
    new_command = 0;

    command_buffer[command_index] = '\0';  // Null-terminate
    total_commands++;

    // Детальный отладочный вывод полученной команды
    Debug_Print(LOG_LEVEL_INFO, "\r\n=== COMMAND DEBUG ===\r\n");
    Debug_Print(LOG_LEVEL_INFO, "Raw command buffer (%d bytes): ", command_index);
    for(int i = 0; i < command_index; i++) {
        Debug_Print(LOG_LEVEL_INFO, "%02X ", command_buffer[i]);
    }
    Debug_Print(LOG_LEVEL_INFO, "\r\n");
    Debug_Print(LOG_LEVEL_INFO, "As string: '%s'\r\n", command_buffer);

    // Удаляем возможные символы \r и \n в конце
    int len = command_index;
    while (len > 0 && (command_buffer[len-1] == '\r' || command_buffer[len-1] == '\n' || command_buffer[len-1] == ' ')) {
        command_buffer[len-1] = '\0';
        len--;
    }
    Debug_Print(LOG_LEVEL_INFO, "Trimmed: '%s' (len=%d)\r\n", command_buffer, strlen((char*)command_buffer));

    char cmd[32];
    // Используем sscanf для извлечения первой команды
    if (sscanf((char*)command_buffer, "%31s", cmd) == 1) {
        Debug_Print(LOG_LEVEL_INFO, "Parsed command: '%s'\r\n", cmd);

        // === HELP ===
        if (strcmp(cmd, CMD_HELP) == 0) {
            Show_Help_Menu();
        }
        else if (strcmp(cmd, "start_stream") == 0) {
            char* args_start = (char*)command_buffer + strlen(cmd);
            while(*args_start == ' ') args_start++;

            if(*args_start != '\0') {
                // Парсим интервал
                int interval = atoi(args_start);
                if(interval >= 10 && interval <= 1000) {
                    stream_interval_ms = interval;
                }
            }

            streaming_active = 1;
            Debug_Print(LOG_LEVEL_INFO, "Streaming started (interval: %lu ms)\r\n", stream_interval_ms);
            Debug_Print(LOG_LEVEL_INFO, "Format: X,Y,Z\\n (in µT)\r\n");
        }
        else if (strcmp(cmd, "stop_stream") == 0) {
            streaming_active = 0;
            Debug_Print(LOG_LEVEL_INFO, "Streaming stopped\r\n");
        }
        else if (strcmp(cmd, "stream_fast") == 0) {
            stream_interval_ms = 10;  // 100 Гц
            streaming_active = 1;
            Debug_Print(LOG_LEVEL_INFO, "Fast streaming started (100 Hz)\r\n");
        }
        else if (strcmp(cmd, "stream_slow") == 0) {
            stream_interval_ms = 100;  // 10 Гц
            streaming_active = 1;
            Debug_Print(LOG_LEVEL_INFO, "Slow streaming started (10 Hz)\r\n");
        }
        else if (strcmp(cmd, "control_test") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== CONTROL SYSTEM TEST ===\r\n");

            // Цель: установить поле в определенной точке
            float target_x = 100.0f;  // µT
            float target_y = 0.0f;
            float target_z = 0.0f;

            Debug_Print(LOG_LEVEL_INFO, "Target field: X=%.1f, Y=%.1f, Z=%.1f µT\r\n",
                       target_x, target_y, target_z);

            // Простой П-регулятор
            float error_x, error_y, error_z;
            float coil_power = 0.0f;
            float Kp = 0.001f;  // Коэффициент усиления

            for(int i = 0; i < 10; i++) {
                // Измеряем текущее поле
                float x_comp, y_comp, z_comp;
                if (Fast_Read_Sensor(0, &x_comp, &y_comp, &z_comp)) {
                    // Вычисляем ошибку
                    error_x = target_x - x_comp;
                    error_y = target_y - y_comp;
                    error_z = target_z - z_comp;

                    // Простое управление одной катушкой по X
                    coil_power += Kp * error_x;
                    if (coil_power < 0) coil_power = 0;
                    if (coil_power > 1) coil_power = 1;

                    Set_Coil_Power(0, coil_power);

                    Debug_Print(LOG_LEVEL_INFO, "Iteration %d: X=%.1f, Error X=%.1f, Coil power=%.3f\r\n",
                               i, x_comp, error_x, coil_power);
                } else {
                    Debug_Print(LOG_LEVEL_ERROR, "Failed to read sensor\r\n");
                }

                HAL_Delay(500);
            }

            // Выключаем катушку
            Set_Coil_Power(0, 0.0f);
            Debug_Print(LOG_LEVEL_INFO, "Control test completed\r\n");
        }
        else if (strcmp(cmd, "check_all") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== CHECKING ALL SENSORS ===\r\n");

            for(int i = 0; i < NUM_SENSORS; i++) {
                if (sensors[i].is_connected) {
                    Debug_Print(LOG_LEVEL_INFO, "Sensor %d: connected\r\n", i);

                    // Можно добавить быстрое измерение
                    // ... код измерения ...
                } else {
                    Debug_Print(LOG_LEVEL_WARNING, "Sensor %d: not connected\r\n", i);
                }
            }
        }
        else if (strcmp(cmd, "calibrate_offset") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== CALIBRATING OFFSET ===\r\n");
            Debug_Print(LOG_LEVEL_INFO, "Make sure no magnets are nearby...\r\n");

            MLX90393_t *sensor = &sensors[0];

            // Среднее нескольких измерений
            float sum_x = 0, sum_y = 0, sum_z = 0;
            int num_samples = 10;

            for(int i = 0; i < num_samples; i++) {
                // Выполняем простое измерение
                uint8_t start_cmd[2] = {0x3F, 0x00};
                uint8_t start_resp[2] = {0};

                HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
                HAL_Delay(1);
                HAL_SPI_TransmitReceive(sensor->spi, start_cmd, start_resp, 2, 100);
                HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

                HAL_Delay(20);

                uint8_t read_cmd[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
                uint8_t read_resp[10] = {0};

                HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
                HAL_Delay(1);
                HAL_SPI_TransmitReceive(sensor->spi, read_cmd, read_resp, 10, 100);
                HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

                // Парсим данные
                int16_t x_raw = (read_resp[4] << 8) | read_resp[5];
                int16_t y_raw = (read_resp[6] << 8) | read_resp[7];
                int16_t z_raw = (read_resp[8] << 8) | read_resp[9];

                // Коэффициенты для GAIN_SEL=0
                float lsb_xy = 0.751f;
                float lsb_z = 1.210f;

                float x_ut = x_raw * lsb_xy;
                float y_ut = y_raw * lsb_xy;
                float z_ut = z_raw * lsb_z;

                sum_x += x_ut;
                sum_y += y_ut;
                sum_z += z_ut;

                Debug_Print(LOG_LEVEL_INFO, "Sample %d: X=%.1f, Y=%.1f, Z=%.1f µT\r\n",
                           i, x_ut, y_ut, z_ut);

                HAL_Delay(100);
            }

            // Сохраняем смещения в структуре датчика
            sensor->offset[0] = sum_x / num_samples;
            sensor->offset[1] = sum_y / num_samples;
            sensor->offset[2] = sum_z / num_samples;
            sensor->is_calibrated = 1;

            Debug_Print(LOG_LEVEL_INFO, "Offset calculated: X=%.1f, Y=%.1f, Z=%.1f µT\r\n",
                       sensor->offset[0], sensor->offset[1], sensor->offset[2]);
        }
        else if (strcmp(cmd, "read_compensated") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== COMPENSATED READ ===\r\n");

            MLX90393_t *sensor = &sensors[0];

            // Прочитать датчик (аналогично read0_correct)
            uint8_t start_cmd[2] = {0x3F, 0x00};
            uint8_t start_resp[2] = {0};

            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(sensor->spi, start_cmd, start_resp, 2, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            HAL_Delay(20);

            uint8_t read_cmd[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
            uint8_t read_resp[10] = {0};

            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(sensor->spi, read_cmd, read_resp, 10, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            // Парсим данные
            int16_t t_raw = (read_resp[2] << 8) | read_resp[3];
            int16_t x_raw = (read_resp[4] << 8) | read_resp[5];
            int16_t y_raw = (read_resp[6] << 8) | read_resp[7];
            int16_t z_raw = (read_resp[8] << 8) | read_resp[9];

            // Коэффициенты для GAIN_SEL=0
            float lsb_xy = 0.751f;
            float lsb_z = 1.210f;

            float x_ut = x_raw * lsb_xy;
            float y_ut = y_raw * lsb_xy;
            float z_ut = z_raw * lsb_z;
            float temp = Calculate_Temperature(t_raw);

            // Применяем компенсацию
            float x_comp = x_ut - sensor->offset[0];
            float y_comp = y_ut - sensor->offset[1];
            float z_comp = z_ut - sensor->offset[2];

            float magnitude = sqrtf(x_comp*x_comp + y_comp*y_comp + z_comp*z_comp);

            Debug_Print(LOG_LEVEL_INFO, "Raw: X=%.1f, Y=%.1f, Z=%.1f µT\r\n", x_ut, y_ut, z_ut);
            Debug_Print(LOG_LEVEL_INFO, "Compensated: X=%.1f, Y=%.1f, Z=%.1f µT\r\n",
                       x_comp, y_comp, z_comp);
            Debug_Print(LOG_LEVEL_INFO, "Magnitude: %.1f µT (Earth: ~50 µT)\r\n", magnitude);
            Debug_Print(LOG_LEVEL_INFO, "Temperature: %.1f°C\r\n", temp);
        }
        else if (strcmp(cmd, "coil_influence") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== TESTING COIL INFLUENCE ===\r\n");

            // 1. Измерить с выключенными катушками
            Debug_Print(LOG_LEVEL_INFO, "1. All coils OFF:\r\n");

            // Вызываем команду read_compensated напрямую
            // (пока просто сделаем измерение здесь)
            // MLX90393_t *sensor = &sensors[0];

            // ... код измерения аналогичный read_compensated ...

            // 2. Включить одну катушку на 10%
            Debug_Print(LOG_LEVEL_INFO, "\r\n2. Coil 0 at 10%%:\r\n");
            Set_Coil_Power(0, 0.1f);
            HAL_Delay(500);

            // ... код измерения ...

            // 3. Выключить
            Set_Coil_Power(0, 0.0f);
            HAL_Delay(500);
        }
        else if (strcmp(cmd, "checkmode") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== CHECKING SENSOR MODE ===\r\n");

            // 1. Попробуем прочитать регистр 1 (COMM_MODE)
            uint8_t tx[4] = {0x50, 0x04, 0x00, 0x00}; // RR reg1 (адрес 0x01 << 2 = 0x04)
            uint8_t rx[4] = {0};

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "Read reg1 response: 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                       rx[0], rx[1], rx[2], rx[3]);

            // 2. Проверим, отвечает ли датчик вообще
            for(int i = 0; i < 3; i++) {
                uint8_t nop_cmd[2] = {0x00, 0x00};
                uint8_t nop_resp[2] = {0};

                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                HAL_Delay(1);
                HAL_SPI_TransmitReceive(&hspi1, nop_cmd, nop_resp, 2, 100);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

                Debug_Print(LOG_LEVEL_INFO, "NOP attempt %d: 0x%02X 0x%02X\r\n",
                           i, nop_resp[0], nop_resp[1]);
                HAL_Delay(10);
            }
        }
        else if (strcmp(cmd, "testcoeff") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== TESTING DIFFERENT COEFFICIENTS ===\r\n");

            // Примерные данные из read0
            int16_t t_raw = -19626;
            int16_t x_raw = 1356;
            int16_t y_raw = 32;
            int16_t z_raw = 1364;

            Debug_Print(LOG_LEVEL_INFO, "Raw: t=%d, x=%d, y=%d, z=%d\r\n", t_raw, x_raw, y_raw, z_raw);

            // GAIN_SEL=0 (default)
            float lsb_xy_0 = 0.751f;
            float lsb_z_0 = 1.210f;
            float temp_0 = Calculate_Temperature(t_raw);

            // GAIN_SEL=5 (attempted)
            float lsb_xy_5 = 0.250f;
            float lsb_z_5 = 0.403f;

            Debug_Print(LOG_LEVEL_INFO, "GAIN_SEL=0: X=%.1f, Y=%.1f, Z=%.1f µT, T=%.1f°C\r\n",
                       x_raw * lsb_xy_0, y_raw * lsb_xy_0, z_raw * lsb_z_0, temp_0);
            Debug_Print(LOG_LEVEL_INFO, "GAIN_SEL=5: X=%.1f, Y=%.1f, Z=%.1f µT, T=%.1f°C\r\n",
                       x_raw * lsb_xy_5, y_raw * lsb_xy_5, z_raw * lsb_z_5, temp_0);

            // Земное поле должно быть ~50 µT
            float magnitude_0 = sqrtf(powf(x_raw * lsb_xy_0, 2) + powf(y_raw * lsb_xy_0, 2) + powf(z_raw * lsb_z_0, 2));
            float magnitude_5 = sqrtf(powf(x_raw * lsb_xy_5, 2) + powf(y_raw * lsb_xy_5, 2) + powf(z_raw * lsb_z_5, 2));

            Debug_Print(LOG_LEVEL_INFO, "Magnitude GAIN_SEL=0: %.1f µT\r\n", magnitude_0);
            Debug_Print(LOG_LEVEL_INFO, "Magnitude GAIN_SEL=5: %.1f µT\r\n", magnitude_5);
        }
        else if (strcmp(cmd, "read0_correct") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== READ SENSOR 0 (GAIN_SEL=0) ===\r\n");

            MLX90393_t *sensor = &sensors[0];

            // Простая последовательность без сброса
            uint8_t start_cmd[2] = {0x3F, 0x00}; // Start measurement XYZT
            uint8_t start_resp[2] = {0};

            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(sensor->spi, start_cmd, start_resp, 2, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            HAL_Delay(20); // Ожидание измерения

            uint8_t read_cmd[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
            uint8_t read_resp[10] = {0};

            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(sensor->spi, read_cmd, read_resp, 10, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            // Парсим данные
            int16_t t_raw = (read_resp[2] << 8) | read_resp[3];
            int16_t x_raw = (read_resp[4] << 8) | read_resp[5];
            int16_t y_raw = (read_resp[6] << 8) | read_resp[7];
            int16_t z_raw = (read_resp[8] << 8) | read_resp[9];

            // Коэффициенты для GAIN_SEL=0
            float lsb_xy = 0.751f;
            float lsb_z = 1.210f;

            float x_ut = x_raw * lsb_xy;
            float y_ut = y_raw * lsb_xy;
            float z_ut = z_raw * lsb_z;
            float temp = Calculate_Temperature(t_raw);

            // Вычисляем общую напряженность
            float magnitude = sqrtf(x_ut*x_ut + y_ut*y_ut + z_ut*z_ut);

            Debug_Print(LOG_LEVEL_INFO, "Raw: t=%d, x=%d, y=%d, z=%d\r\n", t_raw, x_raw, y_raw, z_raw);
            Debug_Print(LOG_LEVEL_INFO, "Field: X=%.1f, Y=%.1f, Z=%.1f µT\r\n", x_ut, y_ut, z_ut);
            Debug_Print(LOG_LEVEL_INFO, "Magnitude: %.1f µT (Earth: ~50 µT)\r\n", magnitude);
            Debug_Print(LOG_LEVEL_INFO, "Temperature: %.1f°C (expected: 25°C)\r\n", temp);
            Debug_Print(LOG_LEVEL_INFO, "Raw temperature: %d (0x%04X)\r\n", t_raw, t_raw);
            Debug_Print(LOG_LEVEL_INFO, "Temp calculation: (%d - 46244) / 45.2 + 25\r\n", t_raw);
        }
        else if (strcmp(cmd, "testmagnet") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== TEST WITH MAGNET ===\r\n");
            Debug_Print(LOG_LEVEL_INFO, "Place magnet near sensor and press any key...\r\n");

            // Читаем без магнита
            system("read0_correct");

            Debug_Print(LOG_LEVEL_INFO, "Now place magnet and press any key...\r\n");
            // Здесь можно добавить ожидание ввода

            // Читаем с магнитом
            system("read0_correct");
        }
        else if (strcmp(cmd, "read0") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== FORCE READ SENSOR 0 ===\r\n");
            MLX90393_t *sensor = &sensors[0];

            // 1. NOP для проверки
            uint8_t nop_cmd[2] = {0x00, 0x00};
            uint8_t nop_resp[2] = {0};

            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(sensor->spi, nop_cmd, nop_resp, 2, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "NOP: 0x%02X 0x%02X\r\n", nop_resp[0], nop_resp[1]);

            // 2. Start measurement
            uint8_t start_cmd[2] = {0x3F, 0x00}; // XYZT
            uint8_t start_resp[2] = {0};

            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(sensor->spi, start_cmd, start_resp, 2, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "Start: 0x%02X 0x%02X\r\n", start_resp[0], start_resp[1]);

            // 3. Wait
            HAL_Delay(20);

            // 4. Read measurement
            uint8_t read_cmd[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
            uint8_t read_resp[10] = {0};

            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(sensor->spi, read_cmd, read_resp, 10, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "Data: ");
            for(int i = 0; i < 10; i++) {
                Debug_Print(LOG_LEVEL_INFO, "0x%02X ", read_resp[i]);
            }
            Debug_Print(LOG_LEVEL_INFO, "\r\n");

            // 5. Parse
            int16_t t_raw = (read_resp[2] << 8) | read_resp[3];
            int16_t x_raw = (read_resp[4] << 8) | read_resp[5];
            int16_t y_raw = (read_resp[6] << 8) | read_resp[7];
            int16_t z_raw = (read_resp[8] << 8) | read_resp[9];

            Debug_Print(LOG_LEVEL_INFO, "Raw: t=%d, x=%d, y=%d, z=%d\r\n", t_raw, x_raw, y_raw, z_raw);

            // 6. Convert
            float lsb_xy = 0.250f; // GAIN_SEL=5
            float lsb_z = 0.403f;
            float temp = ((float)t_raw - 46244.0f) / 45.2f + 25.0f;

            Debug_Print(LOG_LEVEL_INFO, "Converted: X=%.1f, Y=%.1f, Z=%.1f µT, T=%.1f°C\r\n",
                       x_raw * lsb_xy, y_raw * lsb_xy, z_raw * lsb_z, temp);
        }
        else if (strcmp(cmd, "debugsensor") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== SENSOR DEBUG ===\r\n");

            // 1. Проверим конфигурацию датчика
            uint8_t tx[4] = {0x50, 0x00, 0x00, 0x00}; // Read register 0
            uint8_t rx[4] = {0};

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

            uint16_t reg0 = (rx[2] << 8) | rx[3];
            Debug_Print(LOG_LEVEL_INFO, "Register 0: 0x%04X\r\n", reg0);
            Debug_Print(LOG_LEVEL_INFO, "GAIN_SEL: %d, HALLCONF: 0x%X\r\n",
                       (reg0 >> 10) & 0x07, (reg0 >> 6) & 0x0F);

            // 2. Несколько последовательных измерений
            for (int i = 0; i < 5; i++) {
                // Start measurement
                uint8_t start_cmd[2] = {0x3F, 0x00};
                uint8_t start_resp[2] = {0};

                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                HAL_Delay(1);
                HAL_SPI_TransmitReceive(&hspi1, start_cmd, start_resp, 2, 100);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

                Debug_Print(LOG_LEVEL_INFO, "Start response %d: 0x%02X 0x%02X\r\n",
                           i, start_resp[0], start_resp[1]);

                // Wait for measurement
                HAL_Delay(20);

                // Read data
                uint8_t read_cmd[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
                uint8_t read_resp[10] = {0};

                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                HAL_Delay(1);
                HAL_SPI_TransmitReceive(&hspi1, read_cmd, read_resp, 10, 100);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

                // Parse
                int16_t t_raw = (read_resp[2] << 8) | read_resp[3];
                int16_t x_raw = (read_resp[4] << 8) | read_resp[5];
                int16_t y_raw = (read_resp[6] << 8) | read_resp[7];
                int16_t z_raw = (read_resp[8] << 8) | read_resp[9];

                Debug_Print(LOG_LEVEL_INFO, "Raw %d: t=%d, x=%d, y=%d, z=%d\r\n",
                           i, t_raw, x_raw, y_raw, z_raw);

                HAL_Delay(100);
            }
        }
        else if (strcmp(cmd, "parse") == 0) {
            // Парсинг последних полученных данных
            Debug_Print(LOG_LEVEL_INFO, "=== PARSING LAST DATA ===\r\n");
            Debug_Print(LOG_LEVEL_INFO, "From read0 command we got:\r\n");
            Debug_Print(LOG_LEVEL_INFO, "0xFF 0x03 0xB4 0x11 0x07 0xD0 0x00 0xA7 0x09 0x11\r\n");

            int16_t t_raw = 0xB411; // 46113
            int16_t x_raw = 0x07D0; // 2000
            int16_t y_raw = 0x00A7; // 167
            int16_t z_raw = 0x0911; // 2321

            // правильные (GAIN_SEL=5, RES=0):
            float lsb_xy = 0.250f;  // µT/LSB для X, Y (из даташита таблица 17)
            float lsb_z = 0.403f;   // µT/LSB для Z

            float x_uT = x_raw * lsb_xy;
            float y_uT = y_raw * lsb_xy;
            float z_uT = z_raw * lsb_z;
            float temp = ((float)t_raw - 46244.0f) / 45.2f + 25.0f;

            Debug_Print(LOG_LEVEL_INFO, "Parsed values:\r\n");
            Debug_Print(LOG_LEVEL_INFO, "Temperature: %d raw -> %.1f °C\r\n", t_raw, temp);
            Debug_Print(LOG_LEVEL_INFO, "X: %d raw -> %.1f µT\r\n", x_raw, x_uT);
            Debug_Print(LOG_LEVEL_INFO, "Y: %d raw -> %.1f µT\r\n", y_raw, y_uT);
            Debug_Print(LOG_LEVEL_INFO, "Z: %d raw -> %.1f µT\r\n", z_raw, z_uT);
        }
        else if (strcmp(cmd, "spiconfig") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== SPI CONFIGURATION ===\r\n");
            Debug_Print(LOG_LEVEL_INFO, "SPI1 Configuration:\r\n");
            Debug_Print(LOG_LEVEL_INFO, "Mode: %s\r\n", hspi1.Init.Mode == SPI_MODE_MASTER ? "MASTER" : "SLAVE");
            Debug_Print(LOG_LEVEL_INFO, "Direction: %s\r\n", hspi1.Init.Direction == SPI_DIRECTION_2LINES ? "2LINES" : "1LINE");
            Debug_Print(LOG_LEVEL_INFO, "DataSize: %d bits\r\n", hspi1.Init.DataSize == SPI_DATASIZE_8BIT ? 8 : 16);
            Debug_Print(LOG_LEVEL_INFO, "CLKPolarity: %s\r\n", hspi1.Init.CLKPolarity == SPI_POLARITY_LOW ? "LOW (0)" : "HIGH (1)");
            Debug_Print(LOG_LEVEL_INFO, "CLKPhase: %s\r\n", hspi1.Init.CLKPhase == SPI_PHASE_1EDGE ? "1EDGE" : "2EDGE");
            Debug_Print(LOG_LEVEL_INFO, "BaudRatePrescaler: %d\r\n", hspi1.Init.BaudRatePrescaler);
            Debug_Print(LOG_LEVEL_INFO, "FirstBit: %s\r\n", hspi1.Init.FirstBit == SPI_FIRSTBIT_MSB ? "MSB" : "LSB");
        }
        // === STATUS ===
        else if (strcmp(cmd, CMD_STATUS) == 0) {
            Show_System_Status();
        }
        else if (strcmp(cmd, "simpletest") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== SIMPLE SENSOR TEST ===\r\n");

            // MLX90393_t *sensor = &sensors[0];

            // 1. NOP command
            uint8_t nop_cmd[2] = {0x00, 0x00};
            uint8_t nop_resp[2] = {0};

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(&hspi1, nop_cmd, nop_resp, 2, 100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "NOP response: 0x%02X 0x%02X\r\n",
                        nop_resp[0], nop_resp[1]);

            // 2. Simple read - попробуем прочитать 1 байт статуса
            uint8_t read_cmd[2] = {0x3F, 0x00}; // Start measurement XYZT
            uint8_t read_resp[2] = {0};

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(&hspi1, read_cmd, read_resp, 2, 100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "Start measurement response: 0x%02X 0x%02X\r\n",
                        read_resp[0], read_resp[1]);

            // 3. Ждем и читаем результат
            HAL_Delay(10);

            uint8_t read_data[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
            uint8_t data_resp[10] = {0};

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_SPI_TransmitReceive(&hspi1, read_data, data_resp, 10, 100);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "Data response: ");
            for(int i = 0; i < 10; i++) {
                Debug_Print(LOG_LEVEL_INFO, "0x%02X ", data_resp[i]);
            }
            Debug_Print(LOG_LEVEL_INFO, "\r\n");

            // 4. Парсим
            if (data_resp[0] != 0xFF) { // Если не все FF
                int16_t t_raw = (data_resp[2] << 8) | data_resp[3];
                int16_t x_raw = (data_resp[4] << 8) | data_resp[5];
                int16_t y_raw = (data_resp[6] << 8) | data_resp[7];
                int16_t z_raw = (data_resp[8] << 8) | data_resp[9];

                Debug_Print(LOG_LEVEL_INFO, "Raw: t=%d, x=%d, y=%d, z=%d\r\n",
                           t_raw, x_raw, y_raw, z_raw);
            }
        }
        else if (strcmp(cmd, "sensordebug") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== SENSOR DEBUG ===\r\n");

            // 1. Полный сброс
            MLX90393_Reset_Sensor(0);

            // 2. Чтение регистра 0
            uint16_t reg0;
            if (MLX90393_Read_Register(0, 0x00, &reg0)) {
                Debug_Print(LOG_LEVEL_INFO, "Reg0: 0x%04X\r\n", reg0);
                Debug_Print(LOG_LEVEL_INFO, "GAIN_SEL: %d, HALLCONF: 0x%X\r\n",
                           (reg0 >> 10) & 0x07, (reg0 >> 6) & 0x0F);
            }

            // 3. Получение сырых данных
            int16_t t_raw, x_raw, y_raw, z_raw;
            if (MLX90393_Get_Raw_Data(0, &t_raw, &x_raw, &y_raw, &z_raw)) {
                Debug_Print(LOG_LEVEL_INFO, "Raw: t=%d, x=%d, y=%d, z=%d\r\n",
                           t_raw, x_raw, y_raw, z_raw);

                // Конвертация
                float lsb_xy = 0.250f;
                float lsb_z = 0.403f;
                float temp = ((float)t_raw - 46244.0f) / 45.2f + 25.0f;

                Debug_Print(LOG_LEVEL_INFO, "Converted: X=%.1f, Y=%.1f, Z=%.1f µT, T=%.1f°C\r\n",
                           x_raw * lsb_xy, y_raw * lsb_xy, z_raw * lsb_z, temp);
            }
        }

        // === TEST_COIL ===
        else if (strcmp(cmd, CMD_TEST_COIL) == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Processing testcoil command\r\n");
            uint8_t idx;
            float start, end, step;
            uint32_t duration;
            char* args = (char*)command_buffer + strlen(cmd);

            // Пропускаем пробелы
            while(*args == ' ') args++;

            int parsed = sscanf(args, "%hhu %f %f %f %lu", &idx, &start, &end, &step, &duration);
            Debug_Print(LOG_LEVEL_INFO, "Parsed %d arguments for testcoil\r\n", parsed);

            if (parsed == 5) {
                Start_Coil_Test(idx, start, end, step, duration);
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Usage: testcoil <idx> <start> <end> <step> <duration>\r\n");
                Debug_Print(LOG_LEVEL_ERROR, "Received: '%s'\r\n", args);
                Debug_Print(LOG_LEVEL_ERROR, "Expected 5 args, got %d\r\n", parsed);
            }
        }
        // === QUICKTEST ===
        else if (strcmp(cmd, "quicktest") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== QUICK SENSOR TEST ===\n");
            Debug_Print(LOG_LEVEL_INFO, "Testing sensor 0...\n");

            // Принудительно читаем сенсор 0
            uint8_t success = Quick_Read_Sensor(0);

            if (success) {
                Debug_Print(LOG_LEVEL_INFO, "Sensor 0 read successful\n");
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Sensor 0 read failed\n");
            }
        }
        // === TEST_SENSOR ===
        else if (strcmp(cmd, CMD_TEST_SENSOR) == 0) {
            uint8_t idx;
            if (sscanf((char*)command_buffer + strlen(cmd) + 1, "%hhu", &idx) == 1) {
                Read_Sensor(idx);
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Usage: testsensor <idx>\r\n");
            }
        }
        else if (strcmp(cmd, "check_spi") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== SPI CONFIGURATION CHECK ===\r\n");
            Debug_Print(LOG_LEVEL_INFO, "SPI1 Instance: 0x%08lX\r\n", (uint32_t)hspi1.Instance);
            Debug_Print(LOG_LEVEL_INFO, "Mode: %s\r\n",
                        (hspi1.Init.Mode == SPI_MODE_MASTER) ? "MASTER" : "SLAVE");
            Debug_Print(LOG_LEVEL_INFO, "Direction: %s\r\n",
                        (hspi1.Init.Direction == SPI_DIRECTION_2LINES) ? "2LINES" : "1LINE");
            Debug_Print(LOG_LEVEL_INFO, "DataSize: %s\r\n",
                        (hspi1.Init.DataSize == SPI_DATASIZE_8BIT) ? "8BIT" : "16BIT");
            Debug_Print(LOG_LEVEL_INFO, "CLKPolarity: %s\r\n",
                        (hspi1.Init.CLKPolarity == SPI_POLARITY_LOW) ? "LOW (CPOL=0)" : "HIGH (CPOL=1)");
            Debug_Print(LOG_LEVEL_INFO, "CLKPhase: %s\r\n",
                        (hspi1.Init.CLKPhase == SPI_PHASE_1EDGE) ? "1EDGE (CPHA=0)" : "2EDGE (CPHA=1)");
            Debug_Print(LOG_LEVEL_INFO, "BaudRatePrescaler: %d\r\n", hspi1.Init.BaudRatePrescaler);

            // Рассчитаем реальную скорость SPI
            uint32_t spi_clock = HAL_RCC_GetPCLK2Freq(); // SPI1 на APB2
            uint32_t baud_div = 2;
            switch(hspi1.Init.BaudRatePrescaler) {
                case SPI_BAUDRATEPRESCALER_2: baud_div = 2; break;
                case SPI_BAUDRATEPRESCALER_4: baud_div = 4; break;
                case SPI_BAUDRATEPRESCALER_8: baud_div = 8; break;
                case SPI_BAUDRATEPRESCALER_16: baud_div = 16; break;
                case SPI_BAUDRATEPRESCALER_32: baud_div = 32; break;
                case SPI_BAUDRATEPRESCALER_64: baud_div = 64; break;
                case SPI_BAUDRATEPRESCALER_128: baud_div = 128; break;
                case SPI_BAUDRATEPRESCALER_256: baud_div = 256; break;
            }
            uint32_t spi_speed = spi_clock / baud_div;
            Debug_Print(LOG_LEVEL_INFO, "SPI Clock: %lu Hz\r\n", spi_clock);
            Debug_Print(LOG_LEVEL_INFO, "SPI Speed: %lu Hz\r\n", spi_speed);

            // Проверим пины
            Debug_Print(LOG_LEVEL_INFO, "CS pin (PC0) state: %d\r\n",
                        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
            Debug_Print(LOG_LEVEL_INFO, "SCK pin (PA5) state: %d\r\n",
                        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
        }
        else if (strcmp(cmd, "sensor_test") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "=== SENSOR SIMPLE TEST ===\r\n");

            // Простой тест: попробуем прочитать регистр 0
            MLX90393_t *sensor = &sensors[0];

            // 1. Сброс
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            uint8_t reset_cmd[2] = {0xF0, 0x00};
            uint8_t reset_resp[2] = {0};
            HAL_SPI_TransmitReceive(sensor->spi, reset_cmd, reset_resp, 2, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
            HAL_Delay(2);

            Debug_Print(LOG_LEVEL_INFO, "Reset response: 0x%02X 0x%02X\r\n",
                        reset_resp[0], reset_resp[1]);

            // 2. Чтение регистра 0
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            uint8_t read_cmd[4] = {0x50, 0x00, 0x00, 0x00}; // Read register 0
            uint8_t read_resp[4] = {0};
            HAL_SPI_TransmitReceive(sensor->spi, read_cmd, read_resp, 4, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "Read register 0 response: ");
            for(int i = 0; i < 4; i++) {
                Debug_Print(LOG_LEVEL_INFO, "0x%02X ", read_resp[i]);
            }
            Debug_Print(LOG_LEVEL_INFO, "\r\n");

            // 3. Простая команда NOP
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
            HAL_Delay(1);
            uint8_t nop_cmd[2] = {0x00, 0x00};
            uint8_t nop_resp[2] = {0};
            HAL_SPI_TransmitReceive(sensor->spi, nop_cmd, nop_resp, 2, 100);
            HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

            Debug_Print(LOG_LEVEL_INFO, "NOP response: 0x%02X 0x%02X\r\n",
                        nop_resp[0], nop_resp[1]);
        }
        // === CALIBRATE ===
        else if (strcmp(cmd, CMD_CALIBRATE) == 0) {
            Calibrate_Sensors_Start();
        }
        // === MONITOR ===
        else if (strcmp(cmd, CMD_MONITOR) == 0) {
            system_state.monitoring_active = 1;
            Debug_Print(LOG_LEVEL_INFO, "Monitoring started.\r\n");
        }
        // === STOP ===
        else if (strcmp(cmd, CMD_STOP) == 0) {
            Stop_All_Coils();
            system_state.monitoring_active = 0;
            Debug_Print(LOG_LEVEL_INFO, "All stopped.\r\n");
        }
        // === RESET ===
        else if (strcmp(cmd, CMD_RESET) == 0) {
            for(int i = 0; i < NUM_COILS; i++) Reset_Coil_Fault(i);
            Debug_Print(LOG_LEVEL_INFO, "Faults reset.\r\n");
        }
        // === COIL ===
        else if (strcmp(cmd, CMD_COIL) == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Processing COIL command\r\n");

            // Создаем копию аргументов для безопасного парсинга
            char args_str[64];
            char* args_start = (char*)command_buffer + strlen(cmd);

            // Пропускаем пробелы после команды
            while(*args_start == ' ') args_start++;

            strncpy(args_str, args_start, sizeof(args_str) - 1);
            args_str[sizeof(args_str) - 1] = '\0';

            Debug_Print(LOG_LEVEL_INFO, "Coil args string: '%s'\r\n", args_str);

            uint8_t idx;
            float power;

            // Пробуем разные форматы парсинга

            // 1. Стандартный: coil 0 0.1
            if (sscanf(args_str, "%hhu %f", &idx, &power) == 2) {
                Debug_Print(LOG_LEVEL_INFO, "Parsed standard: idx=%d, power=%.3f\r\n", idx, power);
                Set_Coil_Power(idx, power);
            }
            // 2. С целыми числами: coil 0 10 (10%)
            else if (sscanf(args_str, "%hhu %d", &idx, (int*)&power) == 2) {
                Debug_Print(LOG_LEVEL_INFO, "Parsed as int: idx=%d, power=%d%%\r\n", idx, (int)power);
                Set_Coil_Power(idx, (float)power / 100.0f);
            }
            // 3. Без десятичной точки: coil 0 1 (100%)
            else if (sscanf(args_str, "%hhu %hhu", &idx, (uint8_t*)&power) == 2) {
                Debug_Print(LOG_LEVEL_INFO, "Parsed as byte: idx=%d, power=%d\r\n", idx, (uint8_t)power);
                Set_Coil_Power(idx, (float)power);
            }
            // 4. Пробуем через strtok
            else {
                Debug_Print(LOG_LEVEL_INFO, "Trying strtok parsing...\r\n");
                char* token = strtok(args_str, " ");
                if (token) {
                    idx = atoi(token);
                    token = strtok(NULL, " ");
                    if (token) {
                        power = atof(token);
                        Debug_Print(LOG_LEVEL_INFO, "Strtok parsed: idx=%d, power=%.3f\r\n", idx, power);
                        Set_Coil_Power(idx, power);
                    } else {
                        Debug_Print(LOG_LEVEL_ERROR, "Usage: coil <idx> <power>\r\n");
                    }
                } else {
                    Debug_Print(LOG_LEVEL_ERROR, "Usage: coil <idx> <power>\r\n");
                }
            }
        }
        // === SENSOR ===
        else if (strcmp(cmd, CMD_SENSOR) == 0) {
            uint8_t idx;
            if (sscanf((char*)command_buffer + strlen(cmd) + 1, "%hhu", &idx) == 1) {
                Read_Sensor(idx);
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Usage: sensor <idx>\r\n");
            }
        }
        // === SPI ===
        else if (strcmp(cmd, CMD_SPI) == 0) {
            Run_SPI_Test_Suite();
        }
        // === EXPORT ===
        else if (strcmp(cmd, CMD_EXPORT) == 0) {
            Export_Coil_Data_CSV();
            Export_Sensor_Data_CSV();
            Export_System_Log();
        }
        // === CONFIG ===
        else if (strcmp(cmd, CMD_CONFIG) == 0) {
            Save_System_Config();
        }
        // === TEST PINS (новая команда) ===
        else if (strcmp(cmd, "testpins") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Testing GPIO pins...\r\n");
            Test_SPI_Pins();
        }
        // === TEST ALL COILS (новая команда) ===
        else if (strcmp(cmd, "testallcoils") == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Testing all coils...\r\n");
            for(int i = 0; i < NUM_COILS; i++) {
                Debug_Print(LOG_LEVEL_INFO, "Coil %d: 10%%\r\n", i);
                Set_Coil_Power(i, 0.1f);
                HAL_Delay(200);
                Set_Coil_Power(i, 0.0f);
                HAL_Delay(100);
            }
            Debug_Print(LOG_LEVEL_INFO, "Test complete\r\n");
        }
        // === SETCOIL (альтернативная команда) ===
        else if (strcmp(cmd, "setcoil") == 0) {
            char* args_start = (char*)command_buffer + strlen(cmd);
            while(*args_start == ' ') args_start++;

            uint8_t idx;
            float power;
            if (sscanf(args_start, "%hhu %f", &idx, &power) == 2) {
                Debug_Print(LOG_LEVEL_INFO, "Setcoil: idx=%d, power=%.3f\r\n", idx, power);
                Set_Coil_Power(idx, power);
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Usage: setcoil <idx> <power>\r\n");
            }
        }
        // === TEST SPI COMMUNICATION ===
        else if (strcmp(cmd, "testspicom") == 0) {
            Test_SPI_Communication();
        }
        // === UNKNOWN COMMAND ===
        else {
            Debug_Print(LOG_LEVEL_ERROR, "Unknown command: '%s'\r\n", cmd);
            Debug_Print(LOG_LEVEL_INFO, "Try 'help' for available commands\r\n");
        }
    } else {
        Debug_Print(LOG_LEVEL_ERROR, "Failed to parse command\r\n");
    }

    // Сброс буфера команды
    command_index = 0;
    memset(command_buffer, 0, sizeof(command_buffer));

    Debug_Print(LOG_LEVEL_INFO, "=== END COMMAND ===\r\n");
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
