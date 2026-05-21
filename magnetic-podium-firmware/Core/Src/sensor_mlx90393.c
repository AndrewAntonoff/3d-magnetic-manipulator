#include "sensor_mlx90393.h"
#include "config.h"
#include "spi.h"
#include "debug_console.h"
#include "main.h"
#include "core_cm7.h"
#include "qspi_flash.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern SPI_HandleTypeDef hspi1;

// ==================== Глобальный массив ====================
MLX90393_t sensors[NUM_SENSORS];

// ==================== Константы (из рабочей версии) ====================
#define MLX_CMD_RESET       0xF0
#define MLX_CMD_EXIT        0x80
#define MLX_CMD_SM_ALL      0x3F   // Start Measurement всех осей + температура
#define MLX_CMD_RM_ALL      0x4F   // Read Measurement всех осей + температура

#define CS_DELAY_US         5      // задержка после CS (в микросекундах)
#define POST_CS_DELAY_US    10     // задержка после деселекта
#define CONVERSION_DELAY_MS 2      // время конверсии (подобрано experimentally)
#define SENSOR_TIMEOUT_MS   50     // таймаут DMA


// Коэффициенты чувствительности (μT/LSB) для GAIN_SEL=7, RES=0 (заводские)
static const float LSBS_XY = 0.150f;
static const float LSBS_Z  = 0.242f;

// ==================== Состояния конечного автомата ====================
typedef enum {
    SENSOR_IDLE,
    SENSOR_START_CONV,
    SENSOR_WAIT_CONV,
    SENSOR_READING,
    SENSOR_COMPLETE
} SensorState_t;

// --- Новые настройки фильтрации ---
#define MEDIAN_WINDOW 3    // Размер окна медианного фильтра
#define FILTER_ALPHA  0.25f // Степень сглаживания (меньше = плавнее)

// Буферы для медианного фильтра (по 3 оси на каждый датчик)
static float median_buf[ACTIVE_SENSORS][3][MEDIAN_WINDOW];
static uint8_t median_idx[ACTIVE_SENSORS] = {0};


static SensorState_t sensor_state = SENSOR_IDLE;
static uint32_t conv_start_tick = 0;
static uint8_t current_read_index = 0;
static volatile uint8_t dma_busy = 0;
static volatile uint8_t dma_error = 0;

// Буферы DMA (выровнены для кэша H7) - размер кратен 32 для SCB_InvalidateDCache_by_Addr
static uint8_t tx_dma[32] __attribute__((aligned(32)));
static uint8_t rx_dma[32] __attribute__((aligned(32)));

// Вспомогательная функция: простая медиана из 3-х значений
static float get_median3(float *buf) {
    float a = buf[0], b = buf[1], c = buf[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

// ==================== Низкоуровневые аппаратные функции ====================
static inline void sensor_select(MLX90393_t *s) {
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < CS_DELAY_US; i++);
}

static inline void sensor_deselect(MLX90393_t *s) {
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < POST_CS_DELAY_US; i++);
}

// ==================== Синхронные функции (для калибровки и отладки) ====================
static uint8_t mlx90393_reset(MLX90393_t *sensor) {
    uint8_t tx[2] = {MLX_CMD_RESET, 0x00};
    uint8_t rx[2];
    sensor_select(sensor);
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100);
    sensor_deselect(sensor);
    HAL_Delay(20);
    return (res == HAL_OK);
}

static uint8_t mlx90393_exit(MLX90393_t *sensor) {
    uint8_t tx[2] = {MLX_CMD_EXIT, 0x00};
    uint8_t rx[2];
    sensor_select(sensor);
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100);
    sensor_deselect(sensor);
    HAL_Delay(10);
    return (res == HAL_OK);
}

// Синхронное чтение одного датчика (возвращает сырые значения в поле magnetic_field)
uint8_t Read_Sensor_Sync(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_connected) return 0;

    mlx90393_exit(s);

    uint8_t tx_sm[2] = {MLX_CMD_SM_ALL, 0x00};
    uint8_t rx_sm[2];
    sensor_select(s);
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(s->spi, tx_sm, rx_sm, 2, 100);
    sensor_deselect(s);
    if (res != HAL_OK) return 0;

    HAL_Delay(CONVERSION_DELAY_MS);

    uint8_t tx_rm[10] = {MLX_CMD_RM_ALL, 0,0,0,0,0,0,0,0,0};
    uint8_t rx_rm[10];
    sensor_select(s);
    res = HAL_SPI_TransmitReceive(s->spi, tx_rm, rx_rm, 10, 100);
    sensor_deselect(s);
    if (res != HAL_OK) return 0;

    if (rx_rm[0] & 0x10) { // бит ошибки
        Debug_Print(LOG_LEVEL_WARNING, "Sensor %d status error 0x%02X\r\n", sensor_idx, rx_rm[0]);
        return 0;
    }

    int16_t xraw = (rx_rm[4] << 8) | rx_rm[5];
    int16_t yraw = (rx_rm[6] << 8) | rx_rm[7];
    int16_t zraw = (rx_rm[8] << 8) | rx_rm[9];
    uint16_t traw = (rx_rm[2] << 8) | rx_rm[3];

    Debug_Print(LOG_LEVEL_INFO, "Sensor %d raw: x=%d, y=%d, z=%d, t=%d\n",
                    sensor_idx, xraw, yraw, zraw, traw);

    s->magnetic_field[0] = xraw * LSBS_XY;
    s->magnetic_field[1] = yraw * LSBS_XY;
    s->magnetic_field[2] = zraw * LSBS_Z;
    s->temperature = ((float)traw - 46244.0f) / 45.2f + 25.0f;

    return 1;
}

// ==================== DMA-функции для быстрого цикла ====================
static void StartAllConversions(void) {
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (!sensors[i].is_connected) continue;
        uint8_t cmd = MLX_CMD_SM_ALL;
        sensor_select(&sensors[i]);
        HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
        sensor_deselect(&sensors[i]);
    }
    conv_start_tick = HAL_GetTick();
}

// Запуск чтения датчика через DMA
void StartSensorRead(uint8_t idx) {
    if (idx >= ACTIVE_SENSORS || !sensors[idx].is_connected) return;

    // === ВОТ НАШЕ СПАСЕНИЕ ===
    // Фиксируем время старта прямо здесь, чтобы таймер обновлялся
    // даже при вызове из фонового прерывания!
    sensors[idx].dma_start_tick = HAL_GetTick();

    memset(tx_dma, 0, sizeof(tx_dma));
    tx_dma[0] = 0x4F; // RM_ALL (Read Measurement)

    // Очистка кэша перед DMA передачей
    SCB_CleanDCache_by_Addr((uint32_t*)tx_dma, 32);

    dma_busy = 1;
    HAL_GPIO_WritePin(sensors[idx].cs_port, sensors[idx].cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_dma, rx_dma, 10);
}

// Обработка данных после DMA
void ProcessSensorData(uint8_t idx) {
    MLX90393_t *s = &sensors[idx];

    // Инвалидация кэша перед чтением из RAM (критично для H7)
    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_dma, 32);

    // 1. Проверка статуса (Бит 4: ошибка, Биты 5-6: насыщение осей)
    uint8_t status = rx_dma[0];
    if (status & 0x10) {
        s->read_error_count++;
        return;
    }
    if (status & 0x60) { // Насыщение по любой из осей
        Debug_Print(LOG_LEVEL_WARNING, "Sensor %d SATURATED! (Base magnet too close)\n", idx);
    }

    // 2. Сборка сырых данных
    int16_t xraw = (rx_dma[4] << 8) | rx_dma[5];
    int16_t yraw = (rx_dma[6] << 8) | rx_dma[7];
    int16_t zraw = (rx_dma[8] << 8) | rx_dma[9];

    // 3. Перевод в микротеслы
    float val[3];
    val[0] = xraw * LSBS_XY;
    val[1] = yraw * LSBS_XY;
    val[2] = zraw * LSBS_Z;

    // 4. Медианная фильтрация (убирает "иголки" от ШИМ)
    uint8_t m_pos = median_idx[idx];
    for (int axis = 0; axis < 3; axis++) {
        median_buf[idx][axis][m_pos] = val[axis];
        float m_val = get_median3(median_buf[idx][axis]);

        // 5. Экспоненциальное сглаживание поверх медианы
        s->magnetic_field[axis] = (m_val * FILTER_ALPHA) + (s->magnetic_field[axis] * (1.0f - FILTER_ALPHA));
    }

    median_idx[idx] = (m_pos + 1) % MEDIAN_WINDOW;
    s->total_reads++;
}

// Callback завершения SPI DMA
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        HAL_GPIO_WritePin(sensors[current_read_index].cs_port, sensors[current_read_index].cs_pin, GPIO_PIN_SET);
        ProcessSensorData(current_read_index);
        dma_busy = 0;

        // Переходим к следующему датчику в цикле
        current_read_index++;
        if (current_read_index < ACTIVE_SENSORS) {
            StartSensorRead(current_read_index);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        // 1. Сбрасываем флаги ошибок в регистрах SPI
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
        __HAL_SPI_CLEAR_FREFLAG(hspi);

        // 2. Принудительно останавливаем DMA, если оно зависло
        HAL_SPI_DMAStop(hspi);

        // 3. Отпускаем CS текущего датчика
        if (current_read_index < ACTIVE_SENSORS) {
            HAL_GPIO_WritePin(sensors[current_read_index].cs_port,
                              sensors[current_read_index].cs_pin, GPIO_PIN_SET);
            sensors[current_read_index].read_error_count++;
        }

        dma_busy = 0;
        dma_error = 1; // Сигнал для Sensors_Tick пропустить этот датчик
    }
}

// ==================== Функция, вызываемая по таймеру (TIM6) ====================
void Sensors_Tick(void) {
    uint32_t now = HAL_GetTick();

    switch (sensor_state) {
        case SENSOR_IDLE:
            sensor_state = SENSOR_START_CONV;
            break;

        case SENSOR_START_CONV:
            StartAllConversions();
            sensor_state = SENSOR_WAIT_CONV;
            break;

        case SENSOR_WAIT_CONV:
            if (now - conv_start_tick >= CONVERSION_DELAY_MS) {
                current_read_index = 0;
                sensor_state = SENSOR_READING;
                StartSensorRead(current_read_index);
            }
            break;

        case SENSOR_READING:
                    // 1. Если была аппаратная ошибка, обязательно сбрасываем флаг занятости
                    if (dma_error) {
                        dma_error = 0;
                        dma_busy = 0; // <--- Важно! Иначе зависнем
                        current_read_index++;
                    }

                    // 2. Если свободен - запускаем чтение
                    if (!dma_busy && current_read_index < ACTIVE_SENSORS) {
                        StartSensorRead(current_read_index);
                    }
                    // 3. ДОБАВЛЕН ELSE! Проверяем таймаут ТОЛЬКО на следующих тактах
                    else if (dma_busy && current_read_index < ACTIVE_SENSORS) {
                        // Обновляем now на случай, если прерывания задержали код
                        now = HAL_GetTick();

                        if (now - sensors[current_read_index].dma_start_tick > SENSOR_TIMEOUT_MS) {
                            Debug_Print(LOG_LEVEL_WARNING, "DMA timeout sensor %d\r\n", current_read_index);
                            HAL_SPI_DMAStop(&hspi1);

                            // Жесткий сброс периферии SPI (помогает от залипаний)
                            __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
                            __HAL_SPI_CLEAR_FREFLAG(&hspi1);

                            sensor_deselect(&sensors[current_read_index]);
                            sensors[current_read_index].read_error_count++;
                            dma_busy = 0;
                            current_read_index++;
                        }
                    }

                    if (current_read_index >= ACTIVE_SENSORS) {
                        sensor_state = SENSOR_COMPLETE;
                    }
                    break;

        case SENSOR_COMPLETE:
            sensor_state = SENSOR_IDLE;
            break;
    }
}

// ==================== Инициализация ====================
void Sensors_Init(void) {
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");

    // CS port/pin table — use defines from main.h so changes propagate automatically
    static GPIO_TypeDef* const cs_ports[NUM_SENSORS] = {
        Sensor1_CS_GPIO_Port, Sensor2_CS_GPIO_Port, Sensor3_CS_GPIO_Port,
        Sensor4_CS_GPIO_Port, Sensor5_CS_GPIO_Port, Sensor6_CS_GPIO_Port,
        Sensor7_CS_GPIO_Port, Sensor8_CS_GPIO_Port
    };
    static const uint16_t cs_pins[NUM_SENSORS] = {
        Sensor1_CS_Pin, Sensor2_CS_Pin, Sensor3_CS_Pin,
        Sensor4_CS_Pin, Sensor5_CS_Pin, Sensor6_CS_Pin,
        Sensor7_CS_Pin, Sensor8_CS_Pin
    };

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].spi = &hspi1;
        sensors[i].cs_port = cs_ports[i];
        sensors[i].cs_pin  = cs_pins[i];

        sensors[i].index = i;
        sensors[i].is_connected = 0;
        sensors[i].gain_sel = 7;
        sensors[i].magnetic_field[0] = sensors[i].magnetic_field[1] = sensors[i].magnetic_field[2] = 0;
        sensors[i].offset[0] = sensors[i].offset[1] = sensors[i].offset[2] = 0;
        sensors[i].scale[0] = sensors[i].scale[1] = sensors[i].scale[2] = 1.0f;
        sensors[i].is_calibrated = 0;
        sensors[i].read_error_count = 0;
        sensors[i].total_reads = 0;
        sensors[i].failed_reads = 0;
        sensors[i].dma_start_tick = 0;
        sensor_deselect(&sensors[i]);
    }

    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        uint8_t tx[2] = {0x00, 0x00};
        uint8_t rx[2] = {0xFF, 0xFF};
        sensor_select(&sensors[i]);
        HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
        sensor_deselect(&sensors[i]);

        if (res == HAL_OK && !(rx[0]==0xFF && rx[1]==0xFF)) {
            sensors[i].is_connected = 1;
            Debug_Print(LOG_LEVEL_INFO, "Sensor %d detected\r\n", i);
            mlx90393_reset(&sensors[i]);
            mlx90393_exit(&sensors[i]);
        } else {
            Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not detected\r\n", i);
        }
    }

    sensor_state = SENSOR_IDLE;
    dma_busy = 0;
    Debug_Print(LOG_LEVEL_INFO, "MLX90393 init done, %d sensors active\r\n", ACTIVE_SENSORS);

    Load_Calibration_From_Flash();
}

// ==================== Публичные функции (обёртки) ====================
uint8_t Read_Sensor(uint8_t sensor_idx) {
    return Read_Sensor_Sync(sensor_idx);
}

uint8_t Read_Sensor_With_Gain(uint8_t sensor_idx) {
    return Read_Sensor_Sync(sensor_idx);
}

uint8_t Read_Sensor_Calibrated(uint8_t sensor_idx) {
    if (!Read_Sensor_Sync(sensor_idx)) return 0;
    MLX90393_t *s = &sensors[sensor_idx];

    // Отладочная печать
    Debug_Print(LOG_LEVEL_INFO, "DBG: rawX=%.1f, offX=%.1f, scaleX=%.1f\n",
                s->magnetic_field[0], s->offset[0], s->scale[0]);

    float cal_x = (s->magnetic_field[0] + s->offset[0]) * s->scale[0];
    float cal_y = (s->magnetic_field[1] + s->offset[1]) * s->scale[1];
    float cal_z = (s->magnetic_field[2] + s->offset[2]) * s->scale[2];
    float mag = sqrtf(cal_x*cal_x + cal_y*cal_y + cal_z*cal_z);
    Debug_Print(LOG_LEVEL_INFO, "Sensor %d calibrated: X=%.1f, Y=%.1f, Z=%.1f µT, magnitude=%.1f µT\r\n",
                sensor_idx, cal_x, cal_y, cal_z, mag);
    return 1;
}

uint8_t Quick_Read_Sensor(uint8_t sensor_idx) {
    return Read_Sensor_Calibrated(sensor_idx);
}

uint8_t Read_All_Sensors(void) {
    uint8_t ok = 1;
    for (int i = 0; i < ACTIVE_SENSORS; i++)
        if (!Read_Sensor_Sync(i)) ok = 0;
    return ok;
}

void Calibrate_Offset_Procedure(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return;
    if (!sensors[sensor_idx].is_connected) {
        Debug_Print(LOG_LEVEL_ERROR, "Sensor %d not connected\r\n", sensor_idx);
        return;
    }
    Debug_Print(LOG_LEVEL_INFO, "Calibrating sensor %d...\r\n", sensor_idx);

    const int samples = 20;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int valid = 0;

    for (int i = 0; i < samples; i++) {
        if (Read_Sensor_Sync(sensor_idx)) {
            sum_x += sensors[sensor_idx].magnetic_field[0];
            sum_y += sensors[sensor_idx].magnetic_field[1];
            sum_z += sensors[sensor_idx].magnetic_field[2];
            valid++;
        }
        HAL_Delay(50);
    }

    if (valid == 0) {
        Debug_Print(LOG_LEVEL_ERROR, "Calibration failed: no valid readings\r\n");
        return;
    }

    sensors[sensor_idx].offset[0] = -sum_x / valid;
    sensors[sensor_idx].offset[1] = -sum_y / valid;
    sensors[sensor_idx].offset[2] = -sum_z / valid;

    // Сбрасываем масштаб в 1.0
    sensors[sensor_idx].scale[0] = 1.0f;
    sensors[sensor_idx].scale[1] = 1.0f;
    sensors[sensor_idx].scale[2] = 1.0f;

    sensors[sensor_idx].is_calibrated = 1;

    Debug_Print(LOG_LEVEL_INFO, "Sensor %d offset: X=%.1f, Y=%.1f, Z=%.1f µT\r\n",
                sensor_idx,
                sensors[sensor_idx].offset[0],
                sensors[sensor_idx].offset[1],
                sensors[sensor_idx].offset[2]);
}

void Calibrate_Sensors_Start(void) {
    Debug_Print(LOG_LEVEL_INFO, "Starting sensor calibration (remove magnet)...\r\n");
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (sensors[i].is_connected) {
            Calibrate_Offset_Procedure(i);
        }
    }
    Debug_Print(LOG_LEVEL_INFO, "Calibration finished.\r\n");
}

// ==================== Вспомогательные функции ====================
float Calculate_Magnetic_Field_Strength(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return 0.0f;
    float bx = sensors[sensor_idx].magnetic_field[0];
    float by = sensors[sensor_idx].magnetic_field[1];
    float bz = sensors[sensor_idx].magnetic_field[2];
    return sqrtf(bx*bx + by*by + bz*bz);
}

uint8_t Test_Sensor_Connection(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return 0;
    return sensors[sensor_idx].is_connected ? 1 : 0;
}

uint8_t Test_Sensor_Connection_Simple(uint8_t sensor_idx) {
    return Test_Sensor_Connection(sensor_idx);
}

uint8_t Get_Sensor_Health_Status(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (s->read_error_count > 10) return 0;
    if (s->total_reads > 0 && (float)s->failed_reads / s->total_reads > 0.2f) return 0;
    return s->is_connected ? 1 : 0;
}

void Get_Sensor_Data_String(char *buffer, uint16_t buffer_size) {
    snprintf(buffer, buffer_size, "Sensors:");
    for (int i = 0; i < NUM_SENSORS; i++) {
        char temp[32];
        snprintf(temp, sizeof(temp), " %.1f %.1f %.1f",
                 sensors[i].magnetic_field[0],
                 sensors[i].magnetic_field[1],
                 sensors[i].magnetic_field[2]);
        strncat(buffer, temp, buffer_size - strlen(buffer) - 1);
    }
}

void Get_Sensor_Stats_String(char *buffer, uint16_t buffer_size) {
    int connected = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
        if (sensors[i].is_connected) connected++;
    snprintf(buffer, buffer_size, "Sensors: %d/%d connected", connected, NUM_SENSORS);
}

float Get_Sensor_Read_Frequency(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return 0.0f;
    if (sensors[sensor_idx].total_reads < 2) return 0.0f;
    if (sensors[sensor_idx].average_read_time_ms <= 0.0f) return 0.0f;
    return 1000.0f / sensors[sensor_idx].average_read_time_ms;
}

// ==================== Калибровочные данные (QSPI) ====================
static uint32_t calculate_calibration_checksum(const CalibrationData_t *data) {
    uint32_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t size = sizeof(CalibrationData_t) - sizeof(data->checksum);
    for (size_t i = 0; i < size; i++) sum += bytes[i];
    return sum;
}

void Save_Calibration_To_Flash(void) {
    Debug_Print(LOG_LEVEL_INFO, "Saving calibration to flash...\r\n");
    CalibrationData_t cal_data;
    cal_data.signature = CALIBRATION_DATA_SIGNATURE;
    cal_data.version = CALIBRATION_DATA_VERSION;
    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < 3; j++) {
            cal_data.offsets[i][j] = sensors[i].offset[j];
            cal_data.scales[i][j] = sensors[i].scale[j];
        }
    }
    cal_data.checksum = calculate_calibration_checksum(&cal_data);
    QSPI_Flash_EraseSector(CALIBRATION_DATA_ADDR);
    QSPI_Flash_WriteBuffer(CALIBRATION_DATA_ADDR, (uint8_t*)&cal_data, sizeof(CalibrationData_t));
    Debug_Print(LOG_LEVEL_INFO, "Calibration data saved to flash at address 0x%08lX.\r\n", CALIBRATION_DATA_ADDR);
}

void Load_Calibration_From_Flash(void) {
    Debug_Print(LOG_LEVEL_INFO, "Loading calibration from flash...\r\n");
    uint32_t flash_addr = CALIBRATION_DATA_ADDR;
    CalibrationData_t cal_data;
    memset(&cal_data, 0, sizeof(CalibrationData_t));
    QSPI_Flash_ReadBuffer(flash_addr, (uint8_t*)&cal_data, sizeof(CalibrationData_t));

    uint8_t is_empty = 1;
    uint8_t *ptr = (uint8_t*)&cal_data;
    for (int i = 0; i < sizeof(CalibrationData_t); i++) {
        if (ptr[i] != 0xFF) { is_empty = 0; break; }
    }
    if (is_empty) {
        Debug_Print(LOG_LEVEL_INFO, "Flash is empty, no calibration data found.\r\n");
        return;
    }
    if (cal_data.signature != CALIBRATION_DATA_SIGNATURE) {
        Debug_Print(LOG_LEVEL_WARNING, "Flash: Invalid signature (0x%08lX != 0x%08lX), skipping load.\r\n",
                    cal_data.signature, CALIBRATION_DATA_SIGNATURE);
        return;
    }
    uint32_t expected_checksum = calculate_calibration_checksum(&cal_data);
    if (cal_data.checksum != expected_checksum) {
        Debug_Print(LOG_LEVEL_WARNING, "Flash: Checksum mismatch (0x%08lX != 0x%08lX), skipping load.\r\n",
                    cal_data.checksum, expected_checksum);
        return;
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < 3; j++) {
            sensors[i].offset[j] = cal_data.offsets[i][j];
            sensors[i].scale[j] = cal_data.scales[i][j];
            // Если scale равен 0 (например, из-за старой версии), исправляем на 1.0
            if (sensors[i].scale[j] == 0.0f) {
                sensors[i].scale[j] = 1.0f;
                Debug_Print(LOG_LEVEL_WARNING, "Scale[%d][%d] was zero, set to 1.0\n", i, j);
            }
        }
        sensors[i].is_calibrated = 1;
    }
    Debug_Print(LOG_LEVEL_INFO, "Calibration data loaded from flash at address 0x%08lX.\r\n", flash_addr);
}

// ==================== Заглушки для неиспользуемых функций ====================
void Test_SPI_Bus(uint8_t testpattern) { (void)testpattern; }
void Measure_SPI_Timing(void) {}
uint8_t Verify_SPI_Communication(void) { return Test_Sensor_Connection(0) ? 1 : 0; }
uint8_t MLX90393_Read_Register(uint8_t sensor_idx, uint8_t addr, uint16_t *value) { return 0; }
void MLX90393_Reset_Sensor(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return;
    mlx90393_reset(&sensors[sensor_idx]);
}
uint8_t MLX90393_Get_Raw_Data(uint8_t sensor_idx, int16_t *x, int16_t *y, int16_t *z, uint16_t *t) {
    if (!Read_Sensor_Sync(sensor_idx)) return 0;
    if (x) *x = (int16_t)(sensors[sensor_idx].magnetic_field[0] / LSBS_XY);
    if (y) *y = (int16_t)(sensors[sensor_idx].magnetic_field[1] / LSBS_XY);
    if (z) *z = (int16_t)(sensors[sensor_idx].magnetic_field[2] / LSBS_Z);
    // температура не возвращается в этом варианте
    return 1;
}

void Get_Calibrated_Sensor_Data(uint8_t idx, float *x, float *y, float *z) {
    if (idx < NUM_SENSORS) {
        *x = sensors[idx].magnetic_field[0];
        *y = sensors[idx].magnetic_field[1];
        *z = sensors[idx].magnetic_field[2];
    } else {
        *x = *y = *z = 0.0f;
    }
}
