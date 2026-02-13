/* USER CODE BEGIN Header */
/*
@file           : main.c
@brief          : Main program body
@attention
Copyright (c) 2024 Your Company.
All rights reserved.
This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
// ... (в начале main.c)
#include "main.h" // <-- main.h теперь содержит extern объявления
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "config.h" // Должно быть до levitation_control.h и sensor_mlx90393.h
#include "coil_driver.h"
#include "sensor_mlx90393.h" // Зависит от config.h
#include "debug_console.h"
#include <stdio.h> // Для snprintf, sscanf
#include <string.h> // Для strcmp, strlen, memset
#include <stdlib.h> // Для atoi
#include "levitation_control.h" // <-- levitation_control.h теперь не требует main.h
#include "qspi_flash.h" // Для инициализации QSPI Flash
#include "usbd_core.h" // Для USB HID
// #include "usbd_custom_hid.h"
#include "usbd_custom_hid_if.h"
#include "usbd_desc.h" // Обычно содержит объявление MX_USB_DEVICE_Init
#include "usb_device.h" // <-- Добавить, если MX_USB_DEVICE_Init объявлено там
#include "quadspi.h" // Если MX_QUADSPI_Init объявлено там и не включено через main.h

// --- УБРАТЬ ОПРЕДЕЛЕНИЯ ИЗ main.c ---
// Удалите строки:
// SystemState_t system_state = { ... };
// OperationMode_t current_mode = MODE_IDLE;
// HID_JoystickReport_TypeDef Joystick_Report = {0};

// ... (остальной код main.c) ...

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Структура HID отчета для джойстика
// УБРАНА: теперь определена в config.h
// #pragma pack(push, 1) // Упаковка структуры без выравнивания
// typedef struct {
//     uint8_t report_id; // 0x01 (если используется ID)
//     int16_t x;         // Signed 16-bit
//     int16_t y;         // Signed 16-bit
//     int16_t z;         // Signed 16-bit
//     int16_t rx;        // Signed 16-bit (Pitch)
//     int16_t ry;        // Signed 16-bit (Yaw)
//     int16_t rz;        // Signed 16-bit (Roll)
//     uint32_t buttons;  // 32 бит для кнопок (LSB first)
// } HID_JoystickReport_TypeDef;
// #pragma pack(pop)

// Глобальные структуры
// Убрано определение SystemState_t, так как оно определено в config.h
// SystemState_t system_state = { ... }; // Убрано, так как определение в config.h

// Буферы
char console_buffer[256]; // Уменьшенный буфер
uint8_t command_buffer[64]; // Уменьшенный буфер команд
uint16_t command_index = 0;
uint8_t RxChar;

// Тайминги
uint32_t last_monitor_time = 0;
uint32_t last_cpu_measure = 0;
uint32_t last_stream_time = 0; // Для потоковой передачи

// Статистика
uint32_t total_commands = 0;
uint32_t idle_counter = 0;

// Флаги
uint8_t new_command = 0;
uint8_t streaming_active = 0;
uint32_t stream_interval_ms = 50;    // Интервал передачи (20 Гц)

// Глобальная переменная для отчета HID
// Определение глобальной переменной
// --- ОПРЕДЕЛЕНИЕ ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ ---

HID_JoystickReport_TypeDef Joystick_Report = {0};

/* USER CODE END PTD */

// ... (остальной код main.c, остальные функции остаются без изменений) ...

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_HELP           "help"
#define CMD_STATUS         "status"
#define CMD_CALIBRATE      "calibrate"
#define CMD_SAVE_CAL       "save_cal"
#define CMD_LOAD_CAL       "load_cal"
#define CMD_SENSOR         "sensor"
#define CMD_START_STREAM   "start_stream"
#define CMD_STOP_STREAM    "stop_stream"
#define CMD_LEVITATE       "levitate"
#define CMD_STOP_LEVITATE  "stop_levitate"
#define CMD_SET_TARGET     "set_target"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Внешняя переменная для датчиков
extern MLX90393_t sensors[NUM_SENSORS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

// Прототипы внутренних функций
void System_Init(void);
void Process_Console_Commands(void);
void Update_System_Status(void);
void Calculate_CPU_Usage(void);
void Update_Uptime(void);
void Show_Help_Menu(void);
void Show_System_Status(void);
void Stream_Sensor_Data(void); // Добавлено
void Error_Handler(void); // Убедитесь, что только одна реализация

// Прототипы для HID
void Calculate_3D_Position(void); // Заглушка для алгоритма позиции
void Send_HID_Report(void);       // Заглушка для отправки отчета

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Инициализация системы
void System_Init(void) {
    // Инициализация отладки (если используется)
    Debug_Init(&huart2);
    Debug_Print(LOG_LEVEL_INFO, "=== Magnetic Manipulator System Initialization ===\r\n");

    // Инициализация катушек
    Coils_Init();
    system_state.coils_enabled = 1;

    // Инициализация датчиков
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");
    Sensors_Init();
    system_state.sensors_enabled = 1;

    // Инициализация QSPI Flash
    if (!QSPI_Flash_Init()) {
        Debug_Print(LOG_LEVEL_ERROR, "Failed to initialize QSPI Flash!\r\n");
        // emergency_stop = 1; // Пример
    } else {
        Debug_Print(LOG_LEVEL_INFO, "QSPI Flash initialized.\r\n");
    }

    // Загрузка калибровки из Flash (через функцию в sensor_mlx90393.c)
    Debug_Print(LOG_LEVEL_INFO, "Loading calibration data...\r\n");
    Load_Calibration_From_Flash(); // Вызов заглушки

    // Запуск PWM для таймеров (если используются для катушек в будущем)
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Пример
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    // и т.д.

    Debug_Print(LOG_LEVEL_INFO, "System initialization complete.\r\n");
}

void Update_System_Status(void) {
    Update_Uptime();
    Calculate_CPU_Usage();
    // Добавьте другие проверки статуса
}
void Check_QSPI(void) {
    uint32_t jedec_id = QSPI_Flash_ReadJEDECID();
    if (jedec_id != 0xFFFFFFFF && jedec_id != 0) {
        Debug_Print(LOG_LEVEL_INFO, "QSPI JEDEC ID: 0x%06lX (OK)\r\n", jedec_id & 0xFFFFFF);
    } else {
        Debug_Print(LOG_LEVEL_ERROR, "QSPI not responding! ID: 0x%08lX\r\n", jedec_id);
    }
}
void Calculate_CPU_Usage(void) {
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - last_cpu_measure;
    if(elapsed >= 1000) {  // Каждую секунду
        // float total_cycles = (SystemCoreClock / 1000) * elapsed;  // Пример
        // float usage = 100.0f - ((idle_counter * 100.0f) / total_cycles);
        float usage = 0.0f; // Заглушка
        system_state.cpu_usage_percent = usage;
        // Debug_Print(LOG_LEVEL_DEBUG, "CPU usage: %.1f%%\r\n", usage); // Удалено
        idle_counter = 0;
        last_cpu_measure = current_time;
    }
}

void Update_Uptime(void) {
    system_state.system_uptime_ms = HAL_GetTick();
}

// Функция для отображения справки
void Show_Help_Menu(void) {
    Debug_Print(LOG_LEVEL_INFO, "Available commands:\r\n");
    Debug_Print(LOG_LEVEL_INFO, "help - Show this menu\r\n");
    Debug_Print(LOG_LEVEL_INFO, "status - Show system status\r\n");
    Debug_Print(LOG_LEVEL_INFO, "sensor <idx> - Read sensor data\r\n");
    Debug_Print(LOG_LEVEL_INFO, "calibrate - Start sensor calibration (remove magnet first)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "save_cal - Save calibration data to flash\r\n");
    Debug_Print(LOG_LEVEL_INFO, "load_cal - Load calibration data from flash\r\n");
    Debug_Print(LOG_LEVEL_INFO, "start_stream [interval_ms] - Start data streaming (default: 50 ms)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stop_stream - Stop data streaming\r\n");
    Debug_Print(LOG_LEVEL_INFO, "levitate - Start levitation control\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stop_levitate - Stop levitation control\r\n");
    Debug_Print(LOG_LEVEL_INFO, "set_target x y z - Set levitation target position\r\n");
}

// Функция для отображения статуса
void Show_System_Status(void) {
    Debug_Print(LOG_LEVEL_INFO, "System status:\r\n");
    Debug_Print(LOG_LEVEL_INFO, "Uptime: %lu ms\r\n", system_state.system_uptime_ms);
    Debug_Print(LOG_LEVEL_INFO, "CPU usage: %.1f%%\r\n", system_state.cpu_usage_percent);
    Debug_Print(LOG_LEVEL_INFO, "Coils enabled: %d\r\n", system_state.coils_enabled);
    Debug_Print(LOG_LEVEL_INFO, "Sensors enabled: %d\r\n", system_state.sensors_enabled);
    Debug_Print(LOG_LEVEL_INFO, "Calibration done: %d\r\n", system_state.calibration_done);
    Debug_Print(LOG_LEVEL_INFO, "Monitoring active: %d\r\n", system_state.monitoring_active);
    Debug_Print(LOG_LEVEL_INFO, "Levitation active: %d\r\n", system_state.levitation_active);
    Debug_Print(LOG_LEVEL_INFO, "Ball position: X=%.2f, Y=%.2f, Z=%.2f\r\n", system_state.ball_position[0], system_state.ball_position[1], system_state.ball_position[2]);

    Get_Sensor_Stats_String(console_buffer, sizeof(console_buffer));
    Debug_Print(LOG_LEVEL_INFO, "%s\r\n", console_buffer);
}

void Stream_Sensor_Data(void) {
    if(!streaming_active) return;
    uint32_t current_time = HAL_GetTick();
    if(current_time - last_stream_time < stream_interval_ms) {
        return;
    }
    last_stream_time = current_time;

    float x, y, z;
    // Используем Quick_Read_Sensor (или Read_Sensor) для одного датчика как пример
    if(Quick_Read_Sensor(0)) { // Используем существующую функцию
        x = sensors[0].magnetic_field[0];
        y = sensors[0].magnetic_field[1];
        z = sensors[0].magnetic_field[2];
        // Формат: X,Y,Z\n
        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "%.1f,%.1f,%.1f\n", x, y, z);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 10);
    }
}

// --- Заглушки для HID ---
void Calculate_3D_Position(void) {
    // Заглушка: вычисление 3D позиции/ориентации шара на основе данных с датчиков
    // и применение калибровки. Требует сложного алгоритма.
    // Пока просто заполняем нулями.
    Joystick_Report.x = 0;
    Joystick_Report.y = 0;
    Joystick_Report.z = 0;
    Joystick_Report.rx = 0;
    Joystick_Report.ry = 0;
    Joystick_Report.rz = 0;
    Joystick_Report.buttons = 0;
}

void Send_HID_Report(void) {
    // Заглушка: вызов функции отправки отчета HID
    // USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&Joystick_Report, sizeof(Joystick_Report));
    // Реализация требует настройки USB HID
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

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
  MX_QUADSPI_Init();
  Check_QSPI();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Инициализация системы
  System_Init();

  // Включаем прием по UART
  HAL_UART_Receive_IT(&huart2, &RxChar, 1);

  // Включаем таймеры (если используются для катушек)
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // Выводим приветствие
  Debug_Print(LOG_LEVEL_INFO, "\r\n");
  Debug_Print(LOG_LEVEL_INFO, "========================================\r\n");
  Debug_Print(LOG_LEVEL_INFO, "   MAGNETIC MANIPULATOR CONTROL SYSTEM\r\n");
  Debug_Print(LOG_LEVEL_INFO, "   Version: 2.0 Cleaned | Coils: %d | Sensors: %d\r\n", NUM_COILS, NUM_SENSORS);
  Debug_Print(LOG_LEVEL_INFO, "========================================\r\n\r\n");

  // Показываем помощь
  Show_Help_Menu();
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n> ", 4, 10);

  // Запускаем системный таймер
  last_monitor_time = HAL_GetTick();
  last_cpu_measure = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Обработка команд
    Process_Console_Commands();

    // Потоковая передача данных (если включена)
    Stream_Sensor_Data();

    // Обновление статуса системы
    Update_System_Status();

    // --- Заглушка для HID ---
    // static uint32_t last_hid_update = 0;
    // const uint32_t hid_interval_ms = 10; // ~100 Hz
    // if (HAL_GetTick() - last_hid_update >= hid_interval_ms) {
    //     Calculate_3D_Position();
    //     Send_HID_Report();
    //     last_hid_update = HAL_GetTick();
    // }
    // --- Конец заглушки ---

    // Небольшая задержка для стабильности
    if(!streaming_active) {
        HAL_Delay(1);
    } else {
        HAL_Delay(0); // Минимальная задержка при потоковой передаче
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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

void Error_Handler(void) {
    Debug_Print(LOG_LEVEL_ERROR, "Fatal error occurred! System halted.\r\n");
    // Мигаем светодиодом (если есть, например, PC13)
    while(1) {
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Пример
        HAL_Delay(500);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Обработка символов
        if (RxChar == '\r' || RxChar == '\n') {
            if (command_index > 0) {
                new_command = 1;
            }
            // Переводим строку в терминале
            HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);
        }
        else if (RxChar == '\b' || RxChar == 127) {   // Backspace
            if (command_index > 0) {
                command_index--;
                // Стираем символ
                HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, 10);
            }
        }
        else if (command_index < sizeof(command_buffer) - 1) {
            command_buffer[command_index++] = RxChar;
            // Эхо: отправляем символ обратно
            HAL_UART_Transmit(&huart2, &RxChar, 1, 10);
        }

        // Продолжаем приём
        HAL_UART_Receive_IT(&huart2, &RxChar, 1);
    }
}

void Process_Console_Commands(void) {
    if (!new_command) return;
    new_command = 0;
    command_buffer[command_index] = '\0';  // Null-terminate
    total_commands++;

    // Удаляем возможные символы \r и \n в конце
    int len = command_index;
    while (len > 0 && (command_buffer[len-1] == '\r' || command_buffer[len-1] == '\n' || command_buffer[len-1] == ' ')) {
        command_buffer[len-1] = '\0';
        len--;
    }

    char cmd[32];
    // Используем sscanf для извлечения первой команды
    if (sscanf((char*)command_buffer, "%31s", cmd) == 1) {

        if (strcmp(cmd, CMD_HELP) == 0) {
            Show_Help_Menu();
        }
        else if (strcmp(cmd, CMD_STATUS) == 0) {
            Show_System_Status();
        }
        else if (strcmp(cmd, CMD_SENSOR) == 0) {
            uint8_t idx;
            if (sscanf((char*)command_buffer + strlen(cmd) + 1, "%hhu", &idx) == 1) {
                if (idx < NUM_SENSORS) {
                    uint8_t result = Read_Sensor(idx);
                    Debug_Print(LOG_LEVEL_INFO, "Read_Sensor(%d) returned %d\r\n", idx, result);
                    if (result) {
                        Debug_Print(LOG_LEVEL_INFO,
                            "Sensor %d: X=%.1f uT, Y=%.1f uT, Z=%.1f uT, T=%.1f C\r\n",
                            idx,
                            sensors[idx].magnetic_field[0],
                            sensors[idx].magnetic_field[1],
                            sensors[idx].magnetic_field[2],
                            sensors[idx].temperature);
                    } else {
                        Debug_Print(LOG_LEVEL_ERROR, "Failed to read sensor %d, err_cnt=%d, fails=%lu/%lu\r\n",
                            idx, sensors[idx].read_error_count,
                            sensors[idx].failed_reads, sensors[idx].total_reads);
                    }
                } else {
                    Debug_Print(LOG_LEVEL_ERROR, "Sensor index out of range (0-%d)\r\n", NUM_SENSORS-1);
                }
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Usage: sensor <idx>\r\n");
            }
        }
        else if (strcmp(cmd, CMD_CALIBRATE) == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Starting calibration (remove magnet first)...\r\n");
            Calibrate_Sensors_Start(); // Используем функцию из sensor_mlx90393.c
            system_state.calibration_done = 1;
        }
        else if (strcmp(cmd, CMD_SAVE_CAL) == 0) {
            Save_Calibration_To_Flash(); // Вызов из sensor_mlx90393.c
        }
        else if (strcmp(cmd, CMD_LOAD_CAL) == 0) {
            Load_Calibration_From_Flash(); // Вызов из sensor_mlx90393.c
        }
        else if (strcmp(cmd, CMD_START_STREAM) == 0) {
            char* args_start = (char*)command_buffer + strlen(cmd);
            while(*args_start == ' ') args_start++;

            if(*args_start != '\0') {
                int interval = atoi(args_start);
                if(interval >= 10 && interval <= 1000) {
                    stream_interval_ms = interval;
                }
            }

            streaming_active = 1;
            Debug_Print(LOG_LEVEL_INFO, "Streaming started (interval: %lu ms)\r\n", stream_interval_ms);
        }
        else if (strcmp(cmd, CMD_STOP_STREAM) == 0) {
            streaming_active = 0;
            Debug_Print(LOG_LEVEL_INFO, "Streaming stopped\r\n");
        }
        else if (strcmp(cmd, CMD_LEVITATE) == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Starting levitation control...\r\n");
            Start_Levitation(); // Используем функцию из levitation_control.c
        }
        else if (strcmp(cmd, CMD_STOP_LEVITATE) == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Stopping levitation control...\r\n");
            Stop_Levitation(); // Используем функцию из levitation_control.c
        }
        else if (strcmp(cmd, CMD_SET_TARGET) == 0) {
            float x, y, z;
            if (sscanf((char*)command_buffer + strlen(cmd) + 1, "%f %f %f", &x, &y, &z) == 3) {
                Debug_Print(LOG_LEVEL_INFO, "Setting target position to X=%.2f, Y=%.2f, Z=%.2f\r\n", x, y, z);
                Set_Levitation_Target(x, y, z); // Используем функцию из levitation_control.c
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Usage: set_target x y z\r\n");
            }
        }
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
    // Показать приглашение для следующей команды
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n> ", 4, 10);
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
