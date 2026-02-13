#include "sensor_mlx90393.h"
#include "config.h" // <-- Добавить, чтобы получить NUM_SENSORS, CalibrationData_t
#include "qspi_flash.h" // <-- Добавить для QSPI_Flash_*
#include "debug_console.h"
#include "spi.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#define CS_DELAY_MS 1   // было 1, оставить
#define POST_CS_DELAY_MS 2 // после CS в 1

/* Глобальный массив датчиков */
MLX90393_t sensors[NUM_SENSORS];
static uint8_t calibration_running = 0;

/* Внутренние прототипы */
static uint8_t  mlx90393_reset(MLX90393_t *sensor);
static uint8_t  mlx90393_exit(MLX90393_t *sensor);
static uint8_t  mlx90393_start_sm_xyzT(MLX90393_t *sensor, uint8_t *status);
static uint8_t  mlx90393_read_rm_xyzT(MLX90393_t *sensor,
                                      int16_t *traw, int16_t *xraw,
                                      int16_t *yraw, int16_t *zraw);

/* ===== Внутренние вспомогательные функции ===== */
static uint8_t mlx90393_reset(MLX90393_t *sensor)
{
    uint8_t tx[2] = {0xF0, 0x00};   /* RT */
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        // Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in RESET"); // Удалено
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(2); /* по даташиту после RT ~1.5 ms */
    // Debug_Print(LOG_LEVEL_INFO, "MLX90393: RESET resp=0x%02X 0x%02X", rx[0], rx[1]); // Удалено
    return 1;
}

static uint8_t mlx90393_exit(MLX90393_t *sensor)
{
    uint8_t tx[2] = {0x80, 0x00};   /* EX */
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        // Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in EXIT"); // Удалено
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(2);
    // Debug_Print(LOG_LEVEL_INFO, "MLX90393: EXIT resp=0x%02X 0x%02X", rx[0], rx[1]); // Удалено
    return 1;
}



static uint8_t mlx90393_start_sm_xyzT(MLX90393_t *sensor, uint8_t *status)
{
    uint8_t tx[2] = {0x3F, 0x00}; /* SM, zyx t = 1111 */
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        // Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in SM"); // Удалено
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    if (status)
        *status = rx[0];
    // mlx90393_log_status(rx[0]); // Удалено
    if (rx[0] & 0x10)
        return 0;
    HAL_Delay(5);
    return 1;
}

static uint8_t mlx90393_read_rm_xyzT(MLX90393_t *sensor,
                                     int16_t *traw, int16_t *xraw,
                                     int16_t *yraw, int16_t *zraw)
{
    uint8_t tx[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
    uint8_t rx[10] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 10, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        // Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in RM"); // Удалено
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    uint8_t status = rx[0];
    // mlx90393_log_status(status); // Удалено
    if (status & 0x10)
    {
        // Debug_Print(LOG_LEVEL_WARNING, "MLX90393: ERROR bit in RM, status=0x%02X", status); // Удалено
        return 0;
    }
    if (traw) *traw = (int16_t)((rx[2] << 8) | rx[3]);
    if (xraw) *xraw = (int16_t)((rx[4] << 8) | rx[5]);
    if (yraw) *yraw = (int16_t)((rx[6] << 8) | rx[7]);
    if (zraw) *zraw = (int16_t)((rx[8] << 8) | rx[9]);
    return 1;
}


/* ===== Публичный API ===== */

void ConfigureSensor(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS)
        return;
    MLX90393_t *sensor = &sensors[sensor_idx];

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(100);
    // Debug_Print(LOG_LEVEL_INFO, "Configuring MLX90393 sensor %d...", sensor_idx); // Удалено

    // 1. Выход из всех режимов (EX)
    mlx90393_exit(sensor);
    // 2. Сброс (RT)
    mlx90393_reset(sensor);
    // 3. Снова выход из всех режимов (EX)
    mlx90393_exit(sensor);

    // 4. Проверка подключения (простая)
    uint8_t tx[2] = {0x00, 0x00};  // NOP command
    uint8_t rx[2] = {0xFF, 0xFF};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK && !(rx[0] == 0xFF && rx[1] == 0xFF)) {
        sensor->is_connected = 1;
        // Debug_Print(LOG_LEVEL_INFO, "Sensor %d detected", sensor_idx); // Удалено
    } else {
        sensor->is_connected = 0;
        // Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not detected", sensor_idx); // Удалено
    }
}

void Sensors_Init(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensors[i].spi       = &hspi1;
        sensors[i].cs_port   = GPIOC;

        // Назначение CS пинов
        switch(i) {
            case 0: sensors[i].cs_pin = Sensor1_CS_Pin; break;
            case 1: sensors[i].cs_pin = Sensor2_CS_Pin; break;
            case 2: sensors[i].cs_pin = Sensor3_CS_Pin; break;
            case 3: sensors[i].cs_pin = Sensor4_CS_Pin; break;
            case 4: sensors[i].cs_pin = Sensor5_CS_Pin; break;
            case 5: sensors[i].cs_pin = Sensor6_CS_Pin; break;
            case 6: sensors[i].cs_pin = Sensor7_CS_Pin; break;
            case 7: sensors[i].cs_pin = Sensor8_CS_Pin; break;
            default: sensors[i].cs_pin = GPIO_PIN_0; break;
        }

        // Устанавливаем CS в высокий уровень
        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);

        // Инициализация полей
        memset(sensors[i].magnetic_field, 0, sizeof(sensors[i].magnetic_field));
        sensors[i].temperature      = 0.0f;
        sensors[i].is_connected     = 0;
        sensors[i].is_calibrated    = 0;
        sensors[i].total_reads      = 0;
        sensors[i].failed_reads     = 0;
        sensors[i].read_error_count = 0;
        sensors[i].last_read_time   = 0;
        sensors[i].average_read_time_ms = 0.0f;
    }

    // Проверяем подключение датчиков с таймаутом
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++)
    {
        uint32_t tickstart = HAL_GetTick();

        // Простая проверка подключения через NOP
        uint8_t tx[2] = {0x00, 0x00};
        uint8_t rx[2] = {0xFF, 0xFF};

        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);

        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(sensors[i].spi, tx, rx, 2, 100); // Таймаут 100ms

        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);

        if (status == HAL_OK && !(rx[0] == 0xFF && rx[1] == 0xFF)) {
            sensors[i].is_connected = 1;
            Debug_Print(LOG_LEVEL_INFO, "Sensor %d detected\r\n", i);
            ConfigureSensor(i);
        } else {
            sensors[i].is_connected = 0;
            Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not detected (status: %d, rx: %02X %02X)\r\n",
                       i, status, rx[0], rx[1]);
        }

        uint32_t elapsed = HAL_GetTick() - tickstart;
        Debug_Print(LOG_LEVEL_DEBUG, "Sensor %d init took %lu ms\r\n", i, elapsed);
    }

    // Загружаем калибровку из Flash
    Load_Calibration_From_Flash();
}

void Sensors_Deinit(void)
{
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
    }
}

uint8_t Read_Sensor(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS)
        return 0;

    MLX90393_t *sensor = &sensors[sensor_idx];
    if (!sensor->is_connected)
        return 0;

    uint32_t t0 = HAL_GetTick();

    if (!mlx90393_reset(sensor))
    {
        sensor->read_error_count++;
        sensor->failed_reads++;
        // Debug_Print(LOG_LEVEL_ERROR, "Reset failed for sensor %d", sensor_idx); // Удалено
        return 0;
    }

    if (!mlx90393_exit(sensor))
    {
        sensor->read_error_count++;
        sensor->failed_reads++;
        // Debug_Print(LOG_LEVEL_ERROR, "Exit failed for sensor %d", sensor_idx); // Удалено
        return 0;
    }

    uint8_t status = 0;
    if (!mlx90393_start_sm_xyzT(sensor, &status))
    {
        sensor->read_error_count++;
        sensor->failed_reads++;
        // Debug_Print(LOG_LEVEL_ERROR, "Start measurement failed for sensor %d", sensor_idx); // Удалено
        return 0;
    }

    int16_t traw = 0, xraw = 0, yraw = 0, zraw = 0;
    if (!mlx90393_read_rm_xyzT(sensor, &traw, &xraw, &yraw, &zraw))
    {
        sensor->read_error_count++;
        sensor->failed_reads++;
        // Debug_Print(LOG_LEVEL_ERROR, "Read measurement failed for sensor %d", sensor_idx); // Удалено
        return 0;
    }

    float lsb_xy = 0.751f;  // µT/LSB для X, Y (GAIN_SEL=0)
    float lsb_z = 1.210f;   // µT/LSB для Z

    sensor->magnetic_field[0] = xraw * lsb_xy;
    sensor->magnetic_field[1] = yraw * lsb_xy;
    sensor->magnetic_field[2] = zraw * lsb_z;
    sensor->temperature = ((float)traw - 46244.0f) / 45.2f + 25.0f;

    uint32_t t1 = HAL_GetTick();
    sensor->last_read_time = t1;
    sensor->total_reads++;

    if (sensor->total_reads == 1)
    {
        sensor->average_read_time_ms = (float)(t1 - t0);
    }
    else
    {
        float prev = sensor->average_read_time_ms;
        sensor->average_read_time_ms = prev + ((float)(t1 - t0) - prev) / (float)sensor->total_reads;
    }

    // Debug_Print(LOG_LEVEL_INFO, "Sensor %d: X=%.1f, Y=%.1f, Z=%.1f uT, T=%.1f C", sensor_idx, sensor->magnetic_field[0], sensor->magnetic_field[1], sensor->magnetic_field[2], sensor->temperature); // Удалено

    return 1;
}

uint8_t Read_All_Sensors(void)
{
    uint8_t ok = 1;
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++) // Только активные
    {
        if (!Read_Sensor(i))
            ok = 0;
    }
    return ok;
}

void Calibrate_Sensors_Start(void)
{
    calibration_running = 1;
    Debug_Print(LOG_LEVEL_INFO, "Starting sensor calibration..."); // Удалено
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++) // Только активные
    {
        if (Read_Sensor(i))
        {
            for (int j = 0; j < 3; j++)
            {
                sensors[i].offset[j] = -sensors[i].magnetic_field[j]; // Инвертируем для компенсации
                sensors[i].scale[j]  = 1.0f; // Пока только смещение
            }
            sensors[i].is_calibrated = 1;
        }
    }
    Debug_Print(LOG_LEVEL_INFO, "Calibration finished."); // Удалено
}

void Calibrate_Sensors_Stop(void)
{
    calibration_running = 0;
    // Debug_Print(LOG_LEVEL_INFO, "Sensor calibration stopped."); // Удалено
}

uint8_t Is_Calibration_Running(void)
{
    return calibration_running;
}

// --- Реализация функций сохранения и загрузки калибровки ---
uint32_t calculate_calibration_checksum(const CalibrationData_t *data) {
    uint32_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t size = sizeof(CalibrationData_t) - sizeof(data->checksum);

    for(size_t i = 0; i < size; ++i) {
        sum += bytes[i];
    }
    return sum;
}

void Save_Calibration_To_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Saving calibration to flash...\r\n");

    CalibrationData_t cal_data;
    cal_data.signature = CALIBRATION_DATA_SIGNATURE;
    cal_data.version = CALIBRATION_DATA_VERSION;

    for(int i = 0; i < NUM_SENSORS; i++) {
        for(int j = 0; j < 3; j++) {
            cal_data.offsets[i][j] = sensors[i].offset[j];
            cal_data.scales[i][j] = sensors[i].scale[j];
        }
    }

    cal_data.checksum = calculate_calibration_checksum(&cal_data);

    // --- РЕАЛИЗАЦИЯ СОХРАНЕНИЯ ---
    // Адрес в Flash, куда будем записывать
    uint32_t flash_addr = CALIBRATION_DATA_ADDR; // Определено в config.h

    // 1. Стереть сектор (QSPI_Flash_EraseSector стирает 4KB)
    QSPI_Flash_EraseSector(flash_addr);

    // 2. Записать данные
    QSPI_Flash_WriteBuffer(flash_addr, (uint8_t*)&cal_data, sizeof(CalibrationData_t));

    Debug_Print(LOG_LEVEL_INFO, "Calibration data saved to flash at address 0x%08lX.\r\n", flash_addr);
}

void Load_Calibration_From_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Loading calibration from flash...\r\n");

    uint32_t flash_addr = CALIBRATION_DATA_ADDR;
    CalibrationData_t cal_data;

    // Очищаем структуру перед чтением
    memset(&cal_data, 0, sizeof(CalibrationData_t));

    // Читаем с таймаутом
    QSPI_Flash_ReadBuffer(flash_addr, (uint8_t*)&cal_data, sizeof(CalibrationData_t));

    // Проверяем, что флеш не пустая (не все 0xFF)
    uint8_t is_empty = 1;
    uint8_t *ptr = (uint8_t*)&cal_data;
    for(int i = 0; i < sizeof(CalibrationData_t); i++) {
        if(ptr[i] != 0xFF) {
            is_empty = 0;
            break;
        }
    }

    if(is_empty) {
        Debug_Print(LOG_LEVEL_INFO, "Flash is empty, no calibration data found.\r\n");
        return;
    }

    if(cal_data.signature != CALIBRATION_DATA_SIGNATURE) {
        Debug_Print(LOG_LEVEL_WARNING, "Flash: Invalid signature (0x%08lX != 0x%08lX), skipping load.\r\n",
                   cal_data.signature, CALIBRATION_DATA_SIGNATURE);
        return;
    }

    // 3. Проверить контрольную сумму
    uint32_t expected_checksum = calculate_calibration_checksum(&cal_data);
    if(cal_data.checksum != expected_checksum) {
        Debug_Print(LOG_LEVEL_WARNING, "Flash: Checksum mismatch (0x%08lX != 0x%08lX), skipping load.\r\n", cal_data.checksum, expected_checksum);
        return;
    }

    // 4. Если всё ок, скопировать offsets и scales в глобальный массив sensors
    for(int i = 0; i < NUM_SENSORS; i++) {
        for(int j = 0; j < 3; j++) {
            sensors[i].offset[j] = cal_data.offsets[i][j];
            sensors[i].scale[j] = cal_data.scales[i][j];
        }
        sensors[i].is_calibrated = 1; // Помечаем как откалиброванный
    }

    Debug_Print(LOG_LEVEL_INFO, "Calibration data loaded from flash at address 0x%08lX.\r\n", flash_addr);
}

void Apply_Calibration(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS)
        return;
    if (!sensors[sensor_idx].is_calibrated)
        return;

    for (int i = 0; i < 3; i++)
    {
        sensors[sensor_idx].magnetic_field[i] =
            (sensors[sensor_idx].magnetic_field[i] +
             sensors[sensor_idx].offset[i]) *
            sensors[sensor_idx].scale[i];
    }
}

float Calculate_Magnetic_Field_Strength(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS)
        return 0.0f;

    float bx = sensors[sensor_idx].magnetic_field[0];
    float by = sensors[sensor_idx].magnetic_field[1];
    float bz = sensors[sensor_idx].magnetic_field[2];
    return sqrtf(bx * bx + by * by + bz * bz);
}

uint8_t Test_Sensor_Connection(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS)
        return 0;
    return sensors[sensor_idx].is_connected ? 1 : 0;
}

// --- НОВАЯ ФУНКЦИЯ ---
uint8_t Test_Sensor_Connection_Simple(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS)
        return 0;
    // Простая проверка - проверяем флаг is_connected
    // Более сложная проверка может включать отправку NOP команды и проверку ответа
    return sensors[sensor_idx].is_connected ? 1 : 0;
}

uint8_t Get_Sensor_Health_Status(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS)
        return 0;

    MLX90393_t *s = &sensors[sensor_idx];
    if (s->read_error_count > 10)
        return 0;
    if (s->total_reads > 0 &&
        (float)s->failed_reads / (float)s->total_reads > 0.2f)
        return 0;
    return s->is_connected ? 1 : 0;
}

void Get_Sensor_Data_String(char *buffer, uint16_t buffer_size)
{
    snprintf(buffer, buffer_size, "Sensors:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        char temp[32];
        snprintf(temp, sizeof(temp),
                 " %.1f %.1f %.1f",
                 sensors[i].magnetic_field[0],
                 sensors[i].magnetic_field[1],
                 sensors[i].magnetic_field[2]);
        strncat(buffer, temp, buffer_size - strlen(buffer) - 1);
    }
}

void Get_Sensor_Stats_String(char *buffer, uint16_t buffer_size)
{
    int connected = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
        if (sensors[i].is_connected)
            connected++;
    snprintf(buffer, buffer_size,
             "Sensors: %d/%d connected", connected, NUM_SENSORS);
}

float Get_Sensor_Read_Frequency(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS)
        return 0.0f;
    if (sensors[sensor_idx].total_reads < 2)
        return 0.0f;
    if (sensors[sensor_idx].average_read_time_ms <= 0.0f)
        return 0.0f;
    return 1000.0f / sensors[sensor_idx].average_read_time_ms;
}

// --- НОВАЯ ФУНКЦИЯ ---
uint8_t Quick_Read_Sensor(uint8_t sensor_idx)
{
    return Read_Sensor(sensor_idx);
}
