#ifndef __CONFIG_H
#define __CONFIG_H

// Конфигурация системы
#define NUM_COILS         12
#define NUM_SENSORS       8    // Всего пинов сконфигурировано
#define ACTIVE_SENSORS    5    // Реально подключено датчиков

// Маска активных датчиков (битовая маска)
// Датчики 0-4 подключены, 5-7 нет
#define SENSOR_MASK       0x1F  // 0b00011111 (датчики 0,1,2,3,4)

// PWM конфигурация
#define PWM_MAX_VALUE     999.0f
#define PWM_FREQUENCY     1000  // Гц

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

// Геометрия катушек (в мм)
#define COIL_UPPER_RADIUS   45.0f  // Верхний уровень (90мм диаметр)
#define COIL_LOWER_RADIUS   25.0f  // Нижний уровень (50мм диаметр)
#define COIL_HEIGHT_LEVEL1  25.0f  // Высота верхнего уровня от дна
#define COIL_HEIGHT_LEVEL2  5.0f   // Высота нижнего уровня от дна

// Структура состояния системы
typedef struct {
    uint8_t coils_enabled;
    uint8_t sensors_enabled;
    uint8_t monitoring_active;
    uint8_t calibration_done;
    uint32_t system_uptime_ms;
    float cpu_usage_percent;
    uint8_t levitation_active;  // Флаг активного парения
    float ball_position[3];     // Позиция шара (x,y,z) в мм
    float ball_velocity[3];     // Скорость шара
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
    MODE_LEVITATION,    // Режим парения
    MODE_FAULT
} OperationMode_t;

// ПИД параметры для парения
typedef struct {
    float Kp[3];  // X, Y, Z
    float Ki[3];
    float Kd[3];
    float integral[3];
    float prev_error[3];
    float setpoint[3];  // Целевая позиция
    float output[3];    // Выходной сигнал
    float max_integral; // Анти-виндъяп
    float max_output;   // Ограничение выхода
} PID_Controller_t;

#endif /* __CONFIG_H */
