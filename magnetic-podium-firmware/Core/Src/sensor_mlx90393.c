#include "sensor_mlx90393.h"
#include "config.h"
#include "spi.h"
#include "debug_console.h"
#include "main.h"
#include "core_cm7.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

// ------------------------------------------------------------------
// Глобальный массив датчиков
// ------------------------------------------------------------------
MLX90393_t sensors[NUM_SENSORS];

// ------------------------------------------------------------------
// Константы команд MLX90393
// ------------------------------------------------------------------
#define MLX_CMD_SM      0x3F   // Start measurement (single mode)
#define MLX_CMD_RM      0x4F   // Read measurement
#define MLX_CMD_RESET   0xF0   // Reset

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

// ------------------------------------------------------------------
// Параметры DMA
// ------------------------------------------------------------------
#define DMA_BUF_SIZE     16
#define SENSOR_TIMEOUT   5      // мс, таймаут на ответ датчика
#define FILTER_ALPHA     0.3f   // коэффициент IIR-фильтра (0..1)

// Глобальные буферы для DMA (выровнены для кэша)
static uint8_t tx_dma[DMA_BUF_SIZE] __attribute__((aligned(32)));
static uint8_t rx_dma[DMA_BUF_SIZE] __attribute__((aligned(32)));

static volatile uint8_t dma_busy = 0;          // флаг занятости SPI
static volatile uint8_t current_sensor = 0;    // текущий читаемый датчик
static uint32_t conversion_start = 0;           // время начала преобразования
static uint8_t conversion_started = 0;          // флаг, что преобразование запущено

static volatile uint8_t active_sensor = 0xFF;   // индекс датчика, для которого запущена активная DMA-передача (0xFF = нет передачи)

// ------------------------------------------------------------------
// Коэффициенты пересчёта для разных GAIN_SEL (из даташита)
// ------------------------------------------------------------------
static const float gain_coeff[8][2] = {
    {0.751f, 1.210f}, {0.601f, 0.968f}, {0.451f, 0.726f}, {0.376f, 0.605f},
    {0.300f, 0.484f}, {0.250f, 0.403f}, {0.200f, 0.323f}, {0.150f, 0.242f}
};

// ------------------------------------------------------------------
// Вспомогательные функции управления CS
// ------------------------------------------------------------------
static inline void sensor_select(MLX90393_t *s)
{
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_RESET);
}

static inline void sensor_deselect(MLX90393_t *s)
{
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_SET);
}

// ------------------------------------------------------------------
// Простой IIR-фильтр для сглаживания показаний
// ------------------------------------------------------------------
static inline float filter(float old, float new_val)
{
    return old * (1.0f - FILTER_ALPHA) + new_val * FILTER_ALPHA;
}

// ------------------------------------------------------------------
// Инициализация всех датчиков (с проверкой связи и установкой GAIN_SEL)
// ------------------------------------------------------------------
void Sensors_Init(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");

    // Сброс всех состояний
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].spi = &hspi1;
        sensors[i].cs_port = GPIOC;
        sensors[i].is_connected = 0;
        sensors[i].gain_sel = 7;                // по умолчанию GAIN_SEL=7
        sensors[i].magnetic_field[0] = 0;
        sensors[i].magnetic_field[1] = 0;
        sensors[i].magnetic_field[2] = 0;
        sensors[i].offset[0] = 0;
        sensors[i].offset[1] = 0;
        sensors[i].offset[2] = 0;
        sensors[i].scale[0] = 1.0f;
        sensors[i].scale[1] = 1.0f;
        sensors[i].scale[2] = 1.0f;
        sensors[i].is_calibrated = 0;
        sensors[i].total_reads = 0;
        sensors[i].failed_reads = 0;
        sensors[i].dma_state = 0;                // не используется в новой схеме, но оставим
        sensor_deselect(&sensors[i]);
        sensors[i].dma_start_tick = 0;
    }

    // Назначение пинов CS в соответствии с main.h
    sensors[0].cs_pin = Sensor1_CS_Pin;
    sensors[1].cs_pin = Sensor2_CS_Pin;
    sensors[2].cs_pin = Sensor3_CS_Pin;
    sensors[3].cs_pin = Sensor4_CS_Pin;
    sensors[4].cs_pin = Sensor5_CS_Pin;
    sensors[5].cs_pin = Sensor6_CS_Pin;
    sensors[6].cs_pin = Sensor7_CS_Pin;
    sensors[7].cs_pin = Sensor8_CS_Pin;

    HAL_Delay(50);

    // Посылаем команду RESET всем датчикам и проверяем связь
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        uint8_t tx[2] = {MLX_CMD_RESET, 0x00};
        uint8_t rx[2] = {0xFF, 0xFF};

        sensor_select(&sensors[i]);
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
        sensor_deselect(&sensors[i]);

        if (status == HAL_OK && !(rx[0] == 0xFF && rx[1] == 0xFF)) {
            sensors[i].is_connected = 1;
            Debug_Print(LOG_LEVEL_INFO, "Sensor %d detected (factory settings, GAIN_SEL=7)\r\n", i);
            // Можно изменить GAIN_SEL здесь, если нужно, записав в регистр
        } else {
            sensors[i].is_connected = 0;
            Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not detected\r\n", i);
        }
        HAL_Delay(10);
    }

    // Инициализация переменных DMA
    dma_busy = 0;
    current_sensor = 0;
    conversion_started = 0;

    Debug_Print(LOG_LEVEL_INFO, "MLX90393 init done, %d sensors active\r\n", ACTIVE_SENSORS);
}

// ------------------------------------------------------------------
// Запуск измерения на всех датчиках (команда SM)
// ------------------------------------------------------------------
static void start_conversion_all(void)
{
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (!sensors[i].is_connected) continue;
        tx_dma[0] = MLX_CMD_SM;
        tx_dma[1] = 0x00;   // байт, следующий за командой (обычно 0)

        sensor_select(&sensors[i]);
        // Отправляем 2 байта: команда + 0x00 (некоторые датчики требуют)
        HAL_SPI_Transmit(&hspi1, tx_dma, 2, 10);
        sensor_deselect(&sensors[i]);
    }
    conversion_start = HAL_GetTick();
    conversion_started = 1;
}

// ------------------------------------------------------------------
// Запуск чтения одного датчика через DMA
// ------------------------------------------------------------------
static void start_read(uint8_t id)
{
    if (dma_busy) return;
    if (!sensors[id].is_connected) {
        current_sensor = (current_sensor + 1) % ACTIVE_SENSORS;
        return;
    }

    MLX90393_t *s = &sensors[id];

    tx_dma[0] = MLX_CMD_RM;
    tx_dma[1] = 0x00;

    SCB_CleanDCache_by_Addr((uint32_t*)tx_dma, 32);
    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_dma, 32);

    sensor_select(s);

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, tx_dma, rx_dma, 7) != HAL_OK) {
        sensor_deselect(s);
        Debug_Print(LOG_LEVEL_ERROR, "DMA start failed for sensor %d\r\n", id);
        s->dma_state = 0;
        dma_busy = 0;
        active_sensor = 0xFF;               // <-- добавить эту строку
        current_sensor = (current_sensor + 1) % ACTIVE_SENSORS;
        return;
    }

    s->dma_state = 1;
    s->dma_start_tick = HAL_GetTick();
    dma_busy = 1;
    active_sensor = id;                      // запоминаем, какой датчик запущен
}

// ------------------------------------------------------------------
// Обработка полученных данных от датчика
// ------------------------------------------------------------------
static void process_sensor(uint8_t id)
{
    MLX90393_t *s = &sensors[id];

    // Структура ответа (7 байт):
    // байт0: статус (обычно не используется)
    // байт1-2: X (старший, младший)
    // байт3-4: Y
    // байт5-6: Z
    int16_t x_raw = (rx_dma[1] << 8) | rx_dma[2];
    int16_t y_raw = (rx_dma[3] << 8) | rx_dma[4];
    int16_t z_raw = (rx_dma[5] << 8) | rx_dma[6];

    // Выбор коэффициентов в зависимости от текущего GAIN_SEL
    uint8_t gain = (s->gain_sel <= 7) ? s->gain_sel : 7;
    float lsb_xy = gain_coeff[gain][0];
    float lsb_z  = gain_coeff[gain][1];

    // Преобразование в микротеслы
    float Bx = x_raw * lsb_xy;
    float By = y_raw * lsb_xy;
    float Bz = z_raw * lsb_z;

    // Применяем IIR-фильтр для сглаживания
    s->magnetic_field[0] = filter(s->magnetic_field[0], Bx);
    s->magnetic_field[1] = filter(s->magnetic_field[1], By);
    s->magnetic_field[2] = filter(s->magnetic_field[2], Bz);

    s->last_read_time = HAL_GetTick();
    s->total_reads++;
}

// ------------------------------------------------------------------
// Колбэк завершения SPI DMA
// ------------------------------------------------------------------
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;
    if (active_sensor == 0xFF) return; // защиты от ложного вызова

    SCB_InvalidateDCache_by_Addr((uint32_t*)rx_dma, 32);

    MLX90393_t *s = &sensors[active_sensor];
    sensor_deselect(s);

    process_sensor(active_sensor);

    s->dma_state = 0;
    dma_busy = 0;

    // Переходим к следующему датчику
    current_sensor = (active_sensor + 1) % ACTIVE_SENSORS;
    active_sensor = 0xFF;
}

// ------------------------------------------------------------------
// Основная функция обновления данных с датчиков (вызывается в главном цикле)
// ------------------------------------------------------------------
void Sensors_Update(void)
{
    uint32_t now = HAL_GetTick();

    // 1. Если преобразование ещё не запущено, запускаем
    if (!conversion_started) {
        start_conversion_all();
        return;
    }

    // 2. Ждём минимум 3 мс после начала преобразования (время измерения)
    if (now - conversion_start < 3)
        return;

    // ------------------------------------------------
    // 3. Проверка согласованности состояний DMA
    //    Если dma_busy=1, но у текущего датчика dma_state != 1,
    //    значит произошёл сбой – принудительно сбрасываем.
    // ------------------------------------------------
    if (dma_busy && sensors[current_sensor].dma_state != 1) {
        Debug_Print(LOG_LEVEL_ERROR, "Inconsistent state: dma_busy=1 but sensor %d dma_state=%d. Resetting.\n",
                    current_sensor, sensors[current_sensor].dma_state);
        HAL_SPI_Abort(&hspi1);
        sensor_deselect(&sensors[current_sensor]);
        sensors[current_sensor].dma_state = 0;
        dma_busy = 0;
        active_sensor = 0xFF;   // <-- добавить эту строку
    }

    // 4. Если SPI свободен, запускаем чтение следующего датчика
    if (!dma_busy) {
        start_read(current_sensor);
    }

    // Отладка состояния (раз в 500 мс)
    static uint32_t last_dbg = 0;
    if (now - last_dbg > 500) {
        last_dbg = now;
        Debug_Print(LOG_LEVEL_INFO, "State: conv_started=%d, dma_busy=%d, curr_sens=%d\n",
                    conversion_started, dma_busy, current_sensor);
    }

    // 5. Проверка таймаута для текущего датчика (если он в состоянии ожидания)
    MLX90393_t *s = &sensors[current_sensor];
    if (s->dma_state == 1) {
        if (now - s->dma_start_tick > SENSOR_TIMEOUT) {
            Debug_Print(LOG_LEVEL_ERROR, "DMA timeout for sensor %d\n", current_sensor);
            HAL_SPI_Abort(&hspi1);
            sensor_deselect(s);
            s->dma_state = 0;
            dma_busy = 0;
            active_sensor = 0xFF;   // <-- добавить эту строку
            current_sensor = (current_sensor + 1) % ACTIVE_SENSORS;
        }
    }

    // 6. Глобальный таймаут: если весь цикл преобразования длится слишком долго
    if (conversion_started && (now - conversion_start > 200)) {
        Debug_Print(LOG_LEVEL_WARNING, "Global conversion timeout, resetting\n");
        conversion_started = 0;
        dma_busy = 0;
        for (int i = 0; i < ACTIVE_SENSORS; i++) {
            sensors[i].dma_state = 0;
        }
        current_sensor = 0;
        active_sensor = 0xFF;   // <-- добавить эту строку
        HAL_SPI_Abort(&hspi1);
    }
}

// ------------------------------------------------------------------
// Синхронное чтение датчика с учётом GAIN_SEL (для калибровки и отладки)
// ------------------------------------------------------------------
uint8_t Read_Sensor_With_Gain(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_connected) return 0;

    uint8_t tx[2] = {MLX_CMD_SM, 0x00};
    uint8_t rx[2];

    // Запуск измерения
    sensor_select(s);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    sensor_deselect(s);
    HAL_Delay(3);   // ждём преобразование

    // Чтение результата
    uint8_t tx_read[7] = {MLX_CMD_RM, 0,0,0,0,0,0};
    uint8_t rx_read[7];
    sensor_select(s);
    HAL_SPI_TransmitReceive(&hspi1, tx_read, rx_read, 7, 100);
    sensor_deselect(s);

    int16_t x_raw = (rx_read[1] << 8) | rx_read[2];
    int16_t y_raw = (rx_read[3] << 8) | rx_read[4];
    int16_t z_raw = (rx_read[5] << 8) | rx_read[6];

    uint8_t gain = (s->gain_sel <= 7) ? s->gain_sel : 7;
    float lsb_xy = gain_coeff[gain][0];
    float lsb_z  = gain_coeff[gain][1];

    s->magnetic_field[0] = x_raw * lsb_xy;
    s->magnetic_field[1] = y_raw * lsb_xy;
    s->magnetic_field[2] = z_raw * lsb_z;

    s->last_read_time = HAL_GetTick();
    s->total_reads++;

    return 1;
}

// ------------------------------------------------------------------
// Калибровка смещения (без шара)
// ------------------------------------------------------------------
void Calibrate_Offset_Procedure(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_connected) {
        Debug_Print(LOG_LEVEL_ERROR, "Sensor %d not connected\r\n", sensor_idx);
        return;
    }

    Debug_Print(LOG_LEVEL_INFO, "Calibrating sensor %d...\r\n", sensor_idx);
    float sum_x = 0, sum_y = 0, sum_z = 0;
    const int samples = 20;

    for (int i = 0; i < samples; i++) {
        if (Read_Sensor_With_Gain(sensor_idx)) {
            sum_x += s->magnetic_field[0];
            sum_y += s->magnetic_field[1];
            sum_z += s->magnetic_field[2];
        }
        HAL_Delay(50);
    }

    s->offset[0] = -sum_x / samples;
    s->offset[1] = -sum_y / samples;
    s->offset[2] = -sum_z / samples;
    s->is_calibrated = 1;

    Debug_Print(LOG_LEVEL_INFO, "Offset: X=%.1f, Y=%.1f, Z=%.1f µT\r\n",
                s->offset[0], s->offset[1], s->offset[2]);
}

// ------------------------------------------------------------------
// Чтение с калибровкой
// ------------------------------------------------------------------
uint8_t Read_Sensor_Calibrated(uint8_t sensor_idx)
{
    if (!Read_Sensor_With_Gain(sensor_idx)) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_calibrated) {
        Debug_Print(LOG_LEVEL_INFO, "Sensor %d not calibrated, raw data displayed.\r\n", sensor_idx);
        return 1;
    }
    float cal_x = s->magnetic_field[0] + s->offset[0];
    float cal_y = s->magnetic_field[1] + s->offset[1];
    float cal_z = s->magnetic_field[2] + s->offset[2];
    float mag = sqrtf(cal_x*cal_x + cal_y*cal_y + cal_z*cal_z);
    Debug_Print(LOG_LEVEL_INFO, "Sensor %d calibrated: X=%.1f, Y=%.1f, Z=%.1f µT, magnitude=%.1f µT\r\n",
                sensor_idx, cal_x, cal_y, cal_z, mag);
    return 1;
}

// ------------------------------------------------------------------
// Вспомогательные функции (для совместимости с main.c)
// ------------------------------------------------------------------
uint8_t Test_Sensor_Connection(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0;
    return sensors[sensor_idx].is_connected;
}

float Calculate_Magnetic_Field_Strength(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0.0f;
    MLX90393_t *s = &sensors[sensor_idx];
    float x = s->magnetic_field[0];
    float y = s->magnetic_field[1];
    float z = s->magnetic_field[2];
    return sqrtf(x*x + y*y + z*z);
}

uint8_t Quick_Read_Sensor(uint8_t sensor_idx)
{
    return Read_Sensor_Calibrated(sensor_idx);
}

uint8_t Read_Sensor(uint8_t sensor_idx)
{
    return Read_Sensor_With_Gain(sensor_idx);
}

void Calibrate_Sensors_Start(void)
{
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++) {
        if (sensors[i].is_connected) {
            Calibrate_Offset_Procedure(i);
        }
    }
}

void Get_Sensor_Stats_String(char *buffer, uint16_t buffer_size)
{
    int connected = 0;
    for (int i = 0; i < ACTIVE_SENSORS; i++)
        if (sensors[i].is_connected) connected++;
    snprintf(buffer, buffer_size, "Sensors: %d/%d connected", connected, ACTIVE_SENSORS);
}

// ------------------------------------------------------------------
// Сохранение/загрузка калибровки во flash (заглушки, при необходимости реализовать)
// ------------------------------------------------------------------
void Save_Calibration_To_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Save_Calibration_To_Flash not implemented\r\n");
}

void Load_Calibration_From_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Load_Calibration_From_Flash not implemented\r\n");
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        Debug_Print(LOG_LEVEL_ERROR, "SPI error on sensor %d, error=%lu\n", current_sensor, hspi->ErrorCode);
        sensors[current_sensor].dma_state = 0;
        dma_busy = 0;
        HAL_SPI_Abort(&hspi1);
        sensor_deselect(&sensors[current_sensor]);
    }
}
