#ifndef __CONFIG_H
#define __CONFIG_H

// Конфигурация системы
#define NUM_COILS         12
#define NUM_SENSORS       8
#define PWM_MAX_VALUE     999.0f
#define PWM_FREQUENCY     1000  // Гц
// config.h - добавьте эти строки
#define ACTIVE_SENSORS 1      // Количество реально подключенных датчиков
#define SENSOR_MASK    0x01   // Маска подключенных датчиков (бит 0 = датчик 0)

// Лимиты безопасности
#define MAX_COIL_CURRENT_MA   2000
#define MAX_DUTY_CYCLE        0.8f
#define MIN_DUTY_CYCLE        0.0f
#define OVERCURRENT_THRESHOLD 2500  // мА

// Конфигурация SPI
#define SPI_TIMEOUT_MS    100
#define SPI_SPEED_HZ      1000000  // 1 МГц

// Конфигурация UART
#define DEBUG_UART        &huart2
#define DEBUG_BAUDRATE    115200

// Структура состояния системы
typedef struct {
    uint8_t coils_enabled;
    uint8_t sensors_enabled;
    uint8_t monitoring_active;
    uint8_t calibration_done;
    uint32_t system_uptime_ms;
    float cpu_usage_percent;
} SystemState_t;

// Перечисление режимов работы
typedef enum {
    MODE_IDLE = 0,
    MODE_TEST_COILS,
    MODE_TEST_SENSORS,
    MODE_CALIBRATION,
    MODE_MONITORING,
    MODE_JOYSTICK,
    MODE_PID_CONTROL,
    MODE_FAULT
} OperationMode_t;

#endif /* __CONFIG_H */
