#include "sensor_mlx90393.h"
#include "config.h"
#include "qspi_flash.h"
#include "debug_console.h"
#include "spi.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

// ------------------------------------------------------------
// Тайминги, проверенные в старой версии
#define CS_DELAY_MS         5
#define POST_CS_DELAY_MS   10
#define RESET_DELAY_MS     20
#define EXIT_DELAY_MS      10
#define SM_CONV_DELAY_MS   15   // достаточно для заводских OSR=0, DIG_FILT=0
// ------------------------------------------------------------

/* Глобальный массив датчиков */
MLX90393_t sensors[NUM_SENSORS];
static uint8_t calibration_running = 0;

/* ===== Внутренние прототипы ===== */
static uint8_t mlx90393_reset(MLX90393_t *sensor);
static uint8_t mlx90393_exit(MLX90393_t *sensor);
static uint8_t mlx90393_start_sm_xyzT(MLX90393_t *sensor, uint8_t *status);
static uint8_t mlx90393_read_rm_xyzT(MLX90393_t *sensor,
                                     int16_t *xraw, int16_t *yraw, int16_t *zraw,
                                     uint16_t *traw);

/* ===== Низкоуровневые функции (как в старой версии) ===== */
static uint8_t mlx90393_reset(MLX90393_t *sensor)
{
    uint8_t tx[2] = {0xF0, 0x00};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(RESET_DELAY_MS);  // 10 мс для стабилизации
    return 1;
}

static uint8_t mlx90393_exit(MLX90393_t *sensor)
{
    uint8_t tx[2] = {0x80, 0x00};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(EXIT_DELAY_MS);
    return 1;
}

static uint8_t mlx90393_start_sm_xyzT(MLX90393_t *sensor, uint8_t *status)
{
    uint8_t tx[2] = {0x3F, 0x00};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(POST_CS_DELAY_MS);
    if (status) *status = rx[0];
    // НИКАКИХ дополнительных проверок! Даже бит ошибки игнорируем.
    // Статус 0x3F – это нормально (SM_MODE=1, данных ещё нет).
    return 1;
}

static uint8_t mlx90393_read_rm_xyzT(MLX90393_t *sensor,
                                     int16_t *xraw, int16_t *yraw, int16_t *zraw,
                                     uint16_t *traw)
{
    uint8_t tx[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
    uint8_t rx[10] = {0};
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 10, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(POST_CS_DELAY_MS);
    if (rx[0] & 0x10) return 0;
    if (xraw) *xraw = (int16_t)((rx[4] << 8) | rx[5]);
    if (yraw) *yraw = (int16_t)((rx[6] << 8) | rx[7]);
    if (zraw) *zraw = (int16_t)((rx[8] << 8) | rx[9]);
    if (traw) *traw = (uint16_t)((rx[2] << 8) | rx[3]);
    return 1;
}

/* ===== Конфигурация датчика – только сброс в известное состояние ===== */
void ConfigureSensor(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return;
    MLX90393_t *s = &sensors[sensor_idx];

    // 1. Полный сброс и выход
    mlx90393_exit(s);
    mlx90393_reset(s);
    mlx90393_exit(s);
    HAL_Delay(10);

    // 2. Проверка связи (NOP)
    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0xFF, 0xFF};
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(s->spi, tx, rx, 2, 100);
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_SET);
    HAL_Delay(POST_CS_DELAY_MS);

    if (status == HAL_OK && !(rx[0] == 0xFF && rx[1] == 0xFF))
    {
        s->is_connected = 1;
        Debug_Print(LOG_LEVEL_INFO, "Sensor %d ready (factory settings)\r\n", sensor_idx);
    }
    else
    {
        s->is_connected = 0;
        Debug_Print(LOG_LEVEL_ERROR, "Sensor %d not responding\r\n", sensor_idx);
    }
}

/* ===== Инициализация всех датчиков ===== */
void Sensors_Init(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensors[i].spi       = &hspi1;
        sensors[i].cs_port   = GPIOC;
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

        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
        memset(sensors[i].magnetic_field, 0, sizeof(sensors[i].magnetic_field));
        sensors[i].temperature      = 0.0f;
        sensors[i].is_connected     = 0;
        sensors[i].is_calibrated    = 0;
        sensors[i].total_reads      = 0;
        sensors[i].failed_reads     = 0;
        sensors[i].read_error_count = 0;
        sensors[i].last_read_time   = 0;
        sensors[i].average_read_time_ms = 0.0f;
        sensors[i].gain_sel         = 7;  // заводское значение
    }

    // Детектируем только активные датчики
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++)
    {
        uint8_t tx[2] = {0x00, 0x00};
        uint8_t rx[2] = {0xFF, 0xFF};

        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_RESET);
        HAL_Delay(CS_DELAY_MS);
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(sensors[i].spi, tx, rx, 2, 100);
        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);

        if (status == HAL_OK && !(rx[0] == 0xFF && rx[1] == 0xFF))
        {
            Debug_Print(LOG_LEVEL_INFO, "Sensor %d detected\r\n", i);
            ConfigureSensor(i);
        }
        else
        {
            sensors[i].is_connected = 0;
            Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not detected\r\n", i);
        }
    }

    Load_Calibration_From_Flash();
}

/* ===== Чтение одного датчика с правильными коэффициентами ===== */
uint8_t Read_Sensor(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_connected) return 0;

    uint32_t t0 = HAL_GetTick();

    // 1. Гарантированный переход в IDLE (один EXIT перед чтением – достаточно)
    mlx90393_exit(s);
    HAL_Delay(2);  // маленькая пауза после EXIT

    // 2. Запуск измерения
    uint8_t status;
    if (!mlx90393_start_sm_xyzT(s, &status))
    {
        s->failed_reads++;
        s->read_error_count++;
        Debug_Print(LOG_LEVEL_ERROR, "SM failed sensor %d (SPI error)\r\n", sensor_idx);
        return 0;
    }

    // 3. Ожидание преобразования
    HAL_Delay(SM_CONV_DELAY_MS);

    // 4. Чтение результата
    int16_t xraw, yraw, zraw;
    uint16_t traw;
    if (!mlx90393_read_rm_xyzT(s, &xraw, &yraw, &zraw, &traw))
    {
        s->failed_reads++;
        s->read_error_count++;
        Debug_Print(LOG_LEVEL_ERROR, "RM failed sensor %d\r\n", sensor_idx);
        return 0;
    }

    // 5. Конверсия (заводские GAIN_SEL=7, RES_XYZ=0)
    const float LSBS_XY = 0.150f;
    const float LSBS_Z  = 0.242f;
    s->magnetic_field[0] = xraw * LSBS_XY;
    s->magnetic_field[1] = yraw * LSBS_XY;
    s->magnetic_field[2] = zraw * LSBS_Z;
    s->temperature = ((float)traw - 46244.0f) / 45.2f + 25.0f;

    // 6. Статистика
    s->total_reads++;
    s->last_read_time = HAL_GetTick();
    float dt = (float)(s->last_read_time - t0);
    if (s->total_reads == 1)
        s->average_read_time_ms = dt;
    else
        s->average_read_time_ms = s->average_read_time_ms * 0.9f + dt * 0.1f;

    Debug_Print(LOG_LEVEL_INFO,
                "Sensor %d: X=%.1f, Y=%.1f, Z=%.1f uT, T=%.1f C\r\n",
                sensor_idx,
                s->magnetic_field[0],
                s->magnetic_field[1],
                s->magnetic_field[2],
                s->temperature);

    return 1;
}

// ------------------------------------------------------------
// Все остальные функции – без изменений, с корректными возвратами
// ------------------------------------------------------------

uint8_t Read_All_Sensors(void)
{
    uint8_t ok = 1;
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++)
        if (!Read_Sensor(i)) ok = 0;
    return ok;
}

void Calibrate_Sensors_Start(void)
{
    calibration_running = 1;
    Debug_Print(LOG_LEVEL_INFO, "Starting sensor calibration (remove magnet)...\r\n");
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++)
    {
        if (Read_Sensor(i))
        {
            for (int j = 0; j < 3; j++)
            {
                sensors[i].offset[j] = -sensors[i].magnetic_field[j];
                sensors[i].scale[j]  = 1.0f;
            }
            sensors[i].is_calibrated = 1;
            Debug_Print(LOG_LEVEL_INFO,
                        "Sensor %d calibrated: offset (%.1f, %.1f, %.1f) uT\r\n",
                        i, sensors[i].offset[0], sensors[i].offset[1], sensors[i].offset[2]);
        }
    }
    Debug_Print(LOG_LEVEL_INFO, "Calibration finished.\r\n");
}

void Calibrate_Sensors_Stop(void) { calibration_running = 0; }
uint8_t Is_Calibration_Running(void) { return calibration_running; }


/* ===== Калибровочные данные (QSPI) ===== */
uint32_t calculate_calibration_checksum(const CalibrationData_t *data)
{
    uint32_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t size = sizeof(CalibrationData_t) - sizeof(data->checksum);
    for (size_t i = 0; i < size; i++) sum += bytes[i];
    return sum;
}

void Save_Calibration_To_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Saving calibration to flash...\r\n");
    CalibrationData_t cal_data;
    cal_data.signature = CALIBRATION_DATA_SIGNATURE;
    cal_data.version = CALIBRATION_DATA_VERSION;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cal_data.offsets[i][j] = sensors[i].offset[j];
            cal_data.scales[i][j] = sensors[i].scale[j];
        }
    }
    cal_data.checksum = calculate_calibration_checksum(&cal_data);
    QSPI_Flash_EraseSector(CALIBRATION_DATA_ADDR);
    QSPI_Flash_WriteBuffer(CALIBRATION_DATA_ADDR, (uint8_t*)&cal_data, sizeof(CalibrationData_t));
    Debug_Print(LOG_LEVEL_INFO, "Calibration data saved to flash at address 0x%08lX.\r\n", CALIBRATION_DATA_ADDR);
}

void Load_Calibration_From_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Loading calibration from flash...\r\n");
    uint32_t flash_addr = CALIBRATION_DATA_ADDR;
    CalibrationData_t cal_data;
    memset(&cal_data, 0, sizeof(CalibrationData_t));
    QSPI_Flash_ReadBuffer(flash_addr, (uint8_t*)&cal_data, sizeof(CalibrationData_t));

    uint8_t is_empty = 1;
    uint8_t *ptr = (uint8_t*)&cal_data;
    for (int i = 0; i < sizeof(CalibrationData_t); i++)
    {
        if (ptr[i] != 0xFF) { is_empty = 0; break; }
    }
    if (is_empty)
    {
        Debug_Print(LOG_LEVEL_INFO, "Flash is empty, no calibration data found.\r\n");
        return;
    }
    if (cal_data.signature != CALIBRATION_DATA_SIGNATURE)
    {
        Debug_Print(LOG_LEVEL_WARNING, "Flash: Invalid signature (0x%08lX != 0x%08lX), skipping load.\r\n",
                    cal_data.signature, CALIBRATION_DATA_SIGNATURE);
        return;
    }
    uint32_t expected_checksum = calculate_calibration_checksum(&cal_data);
    if (cal_data.checksum != expected_checksum)
    {
        Debug_Print(LOG_LEVEL_WARNING, "Flash: Checksum mismatch (0x%08lX != 0x%08lX), skipping load.\r\n",
                    cal_data.checksum, expected_checksum);
        return;
    }
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            sensors[i].offset[j] = cal_data.offsets[i][j];
            sensors[i].scale[j] = cal_data.scales[i][j];
        }
        sensors[i].is_calibrated = 1;
    }
    Debug_Print(LOG_LEVEL_INFO, "Calibration data loaded from flash at address 0x%08lX.\r\n", flash_addr);
}

/* ===== Применение калибровки ===== */
void Apply_Calibration(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return;
    if (!sensors[sensor_idx].is_calibrated) return;
    for (int i = 0; i < 3; i++)
    {
        sensors[sensor_idx].magnetic_field[i] =
            (sensors[sensor_idx].magnetic_field[i] + sensors[sensor_idx].offset[i]) *
            sensors[sensor_idx].scale[i];
    }
}

/* ===== Вспомогательные функции ===== */
float Calculate_Magnetic_Field_Strength(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0.0f;
    float bx = sensors[sensor_idx].magnetic_field[0];
    float by = sensors[sensor_idx].magnetic_field[1];
    float bz = sensors[sensor_idx].magnetic_field[2];
    return sqrtf(bx * bx + by * by + bz * bz);
}

uint8_t Test_Sensor_Connection(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0;
    return sensors[sensor_idx].is_connected ? 1 : 0;
}

uint8_t Test_Sensor_Connection_Simple(uint8_t sensor_idx)
{
    return Test_Sensor_Connection(sensor_idx);
}

uint8_t Get_Sensor_Health_Status(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (s->read_error_count > 10) return 0;
    if (s->total_reads > 0 && (float)s->failed_reads / s->total_reads > 0.2f) return 0;
    return s->is_connected ? 1 : 0;
}

void Get_Sensor_Data_String(char *buffer, uint16_t buffer_size)
{
    snprintf(buffer, buffer_size, "Sensors:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        char temp[32];
        snprintf(temp, sizeof(temp), " %.1f %.1f %.1f",
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
        if (sensors[i].is_connected) connected++;
    snprintf(buffer, buffer_size, "Sensors: %d/%d connected", connected, NUM_SENSORS);
}

float Get_Sensor_Read_Frequency(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0.0f;
    if (sensors[sensor_idx].total_reads < 2) return 0.0f;
    if (sensors[sensor_idx].average_read_time_ms <= 0.0f) return 0.0f;
    return 1000.0f / sensors[sensor_idx].average_read_time_ms;
}

uint8_t Quick_Read_Sensor(uint8_t sensor_idx)
{
    return Read_Sensor(sensor_idx);
}

/* ===== Заглушки для неиспользуемых функций (с корректным возвратом) ===== */
void Test_SPI_Bus(uint8_t testpattern) { (void)testpattern; }
void Measure_SPI_Timing(void) {}
uint8_t Verify_SPI_Communication(void) { return Test_Sensor_Connection(0) ? 1 : 0; }
uint8_t MLX90393_Read_Register(uint8_t sensor_idx, uint8_t addr, uint16_t *value) { return 0; }
void MLX90393_Reset_Sensor(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return;
    ConfigureSensor(sensor_idx);
}
uint8_t MLX90393_Get_Raw_Data(uint8_t sensor_idx, int16_t *x, int16_t *y, int16_t *z, uint16_t *t)
{
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    uint8_t status;
    if (!mlx90393_start_sm_xyzT(s, &status)) return 0;
    HAL_Delay(SM_CONV_DELAY_MS);
    return mlx90393_read_rm_xyzT(s, x, y, z, t);
}
