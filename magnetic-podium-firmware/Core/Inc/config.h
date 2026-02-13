#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h> // Убедитесь, что stdint.h включен

// --- Определения для датчиков ---
#define NUM_SENSORS 8  // Общее количество датчиков (даже если не все используются)
#define ACTIVE_SENSORS 5 // Количество используемых датчиков

// --- Определения для катушек ---
#define NUM_COILS 12   // Общее количество катушек

// --- Определения геометрии ---
#define COIL_UPPER_RADIUS 45.0f  // мм
#define COIL_HEIGHT_LEVEL1 25.0f // мм
#define COIL_LOWER_RADIUS 25.0f  // мм
#define COIL_HEIGHT_LEVEL2 5.0f  // мм

// --- Структура геометрии датчика ---
typedef struct {
    float x, y, z;  // Позиция датчика относительно центра чаши
    float orientation[3];  // Ориентация датчика (вектор нормали) - НЕ ИСПОЛЬЗУЕТСЯ В ТЕКУЩЕМ КОДЕ
} SensorGeometry_t;

// --- Структура геометрии катушки ---
typedef struct {
    float x, y, z;              // Позиция катушки
    float orientation[3];      // Вектор направления силы
    float max_force;           // Максимальная сила (N)
    float resistance_ohm;      // Сопротивление (Ом)
} CoilGeometry_t;

// --- Структура 3D позиции ---
typedef struct {
    float x, y, z;
    float confidence; // Уверенность в позиции
    uint32_t timestamp; // Временная метка
} Position3D_t;

// --- Структура состояния системы ---
typedef struct {
    uint8_t coils_enabled;
    uint8_t sensors_enabled;
    uint8_t monitoring_active;
    uint8_t calibration_done;
    uint8_t levitation_active; // Добавим флаг активности левитации
    uint32_t system_uptime_ms;
    float cpu_usage_percent;
    float ball_position[3]; // Добавим текущую позицию шара
} SystemState_t;

// --- Структура данных калибровки ---
typedef struct {
    uint32_t signature;      // Для проверки валидности
    uint32_t version;        // Для совместимости
    float offsets[NUM_SENSORS][3]; // [X, Y, Z] для каждого датчика
    float scales[NUM_SENSORS][3];   // [X, Y, Z] масштаб для каждого датчика
    // Добавьте другие параметры калибровки, если есть
    uint32_t checksum;       // Простой CRC или сумма для проверки целостности
} CalibrationData_t;

// --- Определения калибровки ---
#define CALIBRATION_DATA_SIGNATURE 0xDEADBEEF
#define CALIBRATION_DATA_VERSION 1
#define CALIBRATION_DATA_ADDR     (0x00000000UL)

// --- Структура HID отчета для джойстика ---
#pragma pack(push, 1) // Упаковка структуры без выравнивания
typedef struct {
    uint8_t report_id; // 0x01 (если используется ID)
    int16_t x;         // Signed 16-bit
    int16_t y;         // Signed 16-bit
    int16_t z;         // Signed 16-bit
    int16_t rx;        // Signed 16-bit (Pitch)
    int16_t ry;        // Signed 16-bit (Yaw)
    int16_t rz;        // Signed 16-bit (Roll)
    uint32_t buttons;  // 32 бит для кнопок (LSB first)
} HID_JoystickReport_TypeDef;
#pragma pack(pop)

// --- Определения для катушек (coil_driver.c) ---
#define PWM_MAX_VALUE 1000.0f // Максимальное значение PWM (например, ARR таймера)
#define OVERCURRENT_THRESHOLD 1000.0f // Порог тока в mA (примерное значение)

// --- Определения PID (levitation_control.c) ---
typedef enum {
    MODE_IDLE,
    MODE_CALIBRATION,
    MODE_LEVITATION,
    MODE_ERROR
} OperationMode_t;

typedef struct {
    float Kp[3]; // X, Y, Z
    float Ki[3];
    float Kd[3];
    float integral[3];
    float prev_error[3];
    float setpoint[3];
    float output[3];
    float max_integral;
    float max_output;
} PID_Controller_t;

// --- Определения для QSPI Flash (qspi_flash.c) ---
// Команды W25Q64JV
#define W25Q_WRITE_ENABLE         0x06
#define W25Q_WRITE_DISABLE        0x04
#define W25Q_READ_STATUS_REG      0x05
#define W25Q_READ_DATA            0x03
#define W25Q_FAST_READ_DATA       0x0B
#define W25Q_PAGE_PROGRAM         0x02
#define W25Q_SECTOR_ERASE         0x20
#define W25Q_CHIP_ERASE           0xC7
#define W25Q_JEDEC_ID             0x9F

#endif /* CONFIG_H */
