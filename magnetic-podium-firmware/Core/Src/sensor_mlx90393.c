#include "sensor_mlx90393.h"
#include "debug_console.h"
#include "spi.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

/* Глобальный массив датчиков */
MLX90393_t sensors[NUM_SENSORS];

static uint8_t calibration_running = 0;

/* Внутренние прототипы */
static uint8_t  mlx90393_reset(MLX90393_t *sensor);
uint8_t Test_Sensor_Connection_Simple(uint8_t sensor_idx);
static uint8_t  mlx90393_exit(MLX90393_t *sensor);
static uint8_t  mlx90393_write_reg(MLX90393_t *sensor, uint8_t addr, uint16_t value);
static uint8_t  mlx90393_read_reg(MLX90393_t *sensor, uint8_t addr, uint16_t *value);
static void     mlx90393_log_status(uint8_t status);
static uint8_t  mlx90393_start_sm_xyzT(MLX90393_t *sensor, uint8_t *status);
static uint8_t  mlx90393_read_rm_xyzT(MLX90393_t *sensor,
                                      int16_t *traw, int16_t *xraw,
                                      int16_t *yraw, int16_t *zraw);

/* ===== Вспомогательные функции ===== */

static void mlx90393_log_status(uint8_t status)
{
    Debug_Print(LOG_LEVEL_INFO,
                "Status 0x%02X | BURST=%d, WOC=%d, SM=%d, ERROR=%d, SED=%d, RS=%d, D=%d",
                status,
                (status & 0x80) ? 1 : 0,
                (status & 0x40) ? 1 : 0,
                (status & 0x20) ? 1 : 0,
                (status & 0x10) ? 1 : 0,
                (status & 0x08) ? 1 : 0,
                (status & 0x04) ? 1 : 0,
                (int)(status & 0x03));
    if (status & 0x10)
    {
        Debug_Print(LOG_LEVEL_WARNING,
                    "MLX90393: ERROR bit set (command rejected / ECC / DRDY low)");
    }
}

static uint8_t mlx90393_reset(MLX90393_t *sensor)
{
    uint8_t tx[2] = {0xF0, 0x00};   /* RT */
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in RESET");
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    HAL_Delay(2); /* по даташиту после RT ~1.5 ms */

    Debug_Print(LOG_LEVEL_INFO, "MLX90393: RESET resp=0x%02X 0x%02X", rx[0], rx[1]);
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
        Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in EXIT");
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    HAL_Delay(2);
    Debug_Print(LOG_LEVEL_INFO, "MLX90393: EXIT resp=0x%02X 0x%02X", rx[0], rx[1]);
    return 1;
}

static uint8_t __attribute__((unused)) mlx90393_write_reg(MLX90393_t *sensor, uint8_t addr, uint16_t value)
{
    uint8_t tx[4] = {
        0x60,                      /* WR */
        (uint8_t)(addr << 2),      /* 0abc = addr<<2 */
        (uint8_t)(value >> 8),     /* D15..8 */
        (uint8_t)(value & 0xFF)    /* D7..0 */
    };
    uint8_t rx[4] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 4, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in WR reg 0x%02X", addr);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    mlx90393_log_status(rx[0]);
    if (rx[0] & 0x10)
        return 0;

    HAL_Delay(2);
    return 1;
}

static uint8_t mlx90393_read_reg(MLX90393_t *sensor, uint8_t addr, uint16_t *value)
{
    uint8_t tx[4] = {
        0x50,                      /* RR */
        (uint8_t)(addr << 2),
        0x00,
        0x00
    };
    uint8_t rx[4] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 4, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in RR reg 0x%02X", addr);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    mlx90393_log_status(rx[0]);
    if (rx[0] & 0x10)
        return 0;

    if (value)
        *value = (uint16_t)((rx[2] << 8) | rx[3]);

    HAL_Delay(1);
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
        Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in SM");
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    if (status)
        *status = rx[0];

    mlx90393_log_status(rx[0]);
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
        Debug_Print(LOG_LEVEL_ERROR, "MLX90393: SPI error in RM");
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    uint8_t status = rx[0];
    mlx90393_log_status(status);
    if (status & 0x10)
    {
        Debug_Print(LOG_LEVEL_WARNING,
                    "MLX90393: ERROR bit in RM, status=0x%02X", status);
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

    Debug_Print(LOG_LEVEL_INFO, "Configuring MLX90393 sensor %d...", sensor_idx);

    // 1. Выход из всех режимов (EX)
    uint8_t exit_cmd[2] = {0x80, 0x00};
    uint8_t exit_resp[2] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(sensor->spi, exit_cmd, exit_resp, 2, 100);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    Debug_Print(LOG_LEVEL_INFO, "EXIT: 0x%02X 0x%02X", exit_resp[0], exit_resp[1]);

    // 2. Сброс (RT)
    uint8_t reset_cmd[2] = {0xF0, 0x00};
    uint8_t reset_resp[2] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(sensor->spi, reset_cmd, reset_resp, 2, 100);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(10);  // Ждем 10 мс после сброса (даташит: 1.5 мс минимум)

    Debug_Print(LOG_LEVEL_INFO, "RESET: 0x%02X 0x%02X", reset_resp[0], reset_resp[1]);

    // 3. Снова выход из всех режимов (EX)
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(sensor->spi, exit_cmd, exit_resp, 2, 100);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(5);

    // 4. Теперь пробуем записать регистр 0
    // GAIN_SEL=5 (0x5), HALLCONF=0xC -> 0x5C00
    uint8_t reg0_cmd[4] = {0x60, 0x00, 0x5C, 0x00}; // WR reg0
    uint8_t reg0_resp[4] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(sensor->spi, reg0_cmd, reg0_resp, 4, 200);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    Debug_Print(LOG_LEVEL_INFO, "WR reg0 SPI status: %d", spi_status);
    Debug_Print(LOG_LEVEL_INFO, "WR reg0 response: 0x%02X 0x%02X 0x%02X 0x%02X",
                reg0_resp[0], reg0_resp[1], reg0_resp[2], reg0_resp[3]);

    // 5. Читаем регистр 0 для проверки
    uint8_t read_reg0_cmd[4] = {0x50, 0x00, 0x00, 0x00}; // RR reg0
    uint8_t read_reg0_resp[4] = {0};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    spi_status = HAL_SPI_TransmitReceive(sensor->spi, read_reg0_cmd, read_reg0_resp, 4, 200);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    Debug_Print(LOG_LEVEL_INFO, "RR reg0 SPI status: %d", spi_status);
    Debug_Print(LOG_LEVEL_INFO, "RR reg0 response: 0x%02X 0x%02X 0x%02X 0x%02X",
                read_reg0_resp[0], read_reg0_resp[1], read_reg0_resp[2], read_reg0_resp[3]);

    uint16_t reg0_val = (read_reg0_resp[2] << 8) | read_reg0_resp[3];
    Debug_Print(LOG_LEVEL_INFO, "Reg0 value: 0x%04X", reg0_val);

    // 6. Проверяем, записалось ли
    if (reg0_val == 0x5C00) {
        sensor->is_connected = 1;
        sensor->gain_sel = 5; // Сохраняем значение GAIN_SEL
        Debug_Print(LOG_LEVEL_INFO, "Sensor %d configured successfully!", sensor_idx);
    } else {
        // Если не записалось, используем настройки по умолчанию
        sensor->is_connected = 1;
        sensor->gain_sel = 0; // GAIN_SEL по умолчанию
        Debug_Print(LOG_LEVEL_WARNING, "Sensor %d: using default settings (GAIN_SEL=0)", sensor_idx);
    }
}

void Sensors_Init(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensors[i].spi       = &hspi1;
        sensors[i].cs_port   = GPIOC;
        // Исправляем назначение CS пинов
        switch(i) {
            case 0: sensors[i].cs_pin = Sensor1_CS_Pin; break;
            case 1: sensors[i].cs_pin = Sensor2_CS_Pin; break;
            case 2: sensors[i].cs_pin = Sensor3_CS_Pin; break;
            case 3: sensors[i].cs_pin = Sensor4_CS_Pin; break;
            case 4: sensors[i].cs_pin = Sensor5_CS_Pin; break;
            case 5: sensors[i].cs_pin = Sensor6_CS_Pin; break;
            case 6: sensors[i].cs_pin = Sensor7_CS_Pin; break;
            case 7: sensors[i].cs_pin = Sensor8_CS_Pin; break;
            default: sensors[i].cs_pin = GPIO_PIN_0;
        }

        sensors[i].i2c_address = 0;
        memset(sensors[i].magnetic_field, 0, sizeof(sensors[i].magnetic_field));
        sensors[i].temperature      = 0.0f;
        sensors[i].is_connected     = 0;
        sensors[i].total_reads      = 0;
        sensors[i].failed_reads     = 0;
        sensors[i].read_error_count = 0;

        // Устанавливаем CS в высокий уровень (неактивен)
        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
    }

    // Проверяем подключение каждого датчика
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++)
    {
        if (Test_Sensor_Connection_Simple(i)) {
            Debug_Print(LOG_LEVEL_INFO, "Sensor %d detected, configuring...\r\n", i);
            ConfigureSensor(i);
        } else {
            Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not detected\r\n", i);
        }
    }

    Load_Calibration_From_Flash();
}

// Добавьте простую функцию проверки подключения
uint8_t Test_Sensor_Connection_Simple(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS)
        return 0;

    MLX90393_t *sensor = &sensors[sensor_idx];

    // Простая проверка: отправляем команду NOP и смотрим ответ
    uint8_t tx[2] = {0x00, 0x00};  // NOP command
    uint8_t rx[2] = {0xFF, 0xFF};  // Инициализируем значением по умолчанию

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100);

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        Debug_Print(LOG_LEVEL_ERROR, "SPI error on sensor %d: %d\r\n", sensor_idx, status);
        return 0;
    }

    // Если ответ не все 0xFF, значит что-то ответило
    // ВАЖНО: даже 0xFF 0x03 - это ответ!
    if (rx[0] == 0xFF && rx[1] == 0xFF) {
        return 0;  // Нет ответа
    }

    Debug_Print(LOG_LEVEL_INFO, "Sensor %d response: 0x%02X 0x%02X\r\n",
                sensor_idx, rx[0], rx[1]);

    // Помечаем как подключенный
    sensor->is_connected = 1;
    return 1;
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
        return 0;
    }
    if (!mlx90393_exit(sensor))
    {
        sensor->read_error_count++;
        sensor->failed_reads++;
        return 0;
    }

    uint8_t status = 0;
    if (!mlx90393_start_sm_xyzT(sensor, &status))
    {
        sensor->read_error_count++;
        sensor->failed_reads++;
        return 0;
    }

    int16_t traw = 0, xraw = 0, yraw = 0, zraw = 0;
    if (!mlx90393_read_rm_xyzT(sensor, &traw, &xraw, &yraw, &zraw))
    {
        sensor->read_error_count++;
        sensor->failed_reads++;
        return 0;
    }

    float lsb_xy = 0.751f;  // µT/LSB для X, Y
    float lsb_z = 1.210f;   // µT/LSB для Z

    sensor->magnetic_field[0] = xraw * lsb_xy;
    sensor->magnetic_field[1] = yraw * lsb_xy;
    sensor->magnetic_field[2] = zraw * lsb_z;

    sensor->temperature =
        ((float)traw - 46244.0f) / 45.2f + 25.0f;

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
        sensor->average_read_time_ms =
            prev + ((float)(t1 - t0) - prev) / (float)sensor->total_reads;
    }

    Debug_Print(LOG_LEVEL_INFO,
                "Sensor %d: X=%.1f, Y=%.1f, Z=%.1f uT, T=%.1f C",
                sensor_idx,
                sensor->magnetic_field[0],
                sensor->magnetic_field[1],
                sensor->magnetic_field[2],
                sensor->temperature);

    return 1;
}

uint8_t Read_All_Sensors(void)
{
    uint8_t ok = 1;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (!Read_Sensor(i))
            ok = 0;
    }
    return ok;
}

void Calibrate_Sensors_Start(void)
{
    calibration_running = 1;
    Debug_Print(LOG_LEVEL_INFO, "Starting sensor calibration...");

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (Read_Sensor(i))
        {
            for (int j = 0; j < 3; j++)
            {
                sensors[i].offset[j] = -sensors[i].magnetic_field[j];
                sensors[i].scale[j]  = 1.0f;
            }
            sensors[i].is_calibrated = 1;
        }
    }
}

void Calibrate_Sensors_Stop(void)
{
    calibration_running = 0;
    Debug_Print(LOG_LEVEL_INFO, "Sensor calibration stopped.");
}

uint8_t Is_Calibration_Running(void)
{
    return calibration_running;
}

void Save_Calibration_To_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Calibration data saved to flash (stub).");
}

void Load_Calibration_From_Flash(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Calibration data loaded from flash (stub).");
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

void Run_Sensor_Self_Test(uint8_t sensor_idx)
{
    Debug_Print(LOG_LEVEL_INFO, "Running self-test for sensor %d...", sensor_idx);
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

// --- SPI test helpers ---

void Test_SPI_Bus(uint8_t testpattern)
{
    // Простейший тест: дернуть одну команду NOP ко всем сенсорам
    (void)testpattern;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (!Test_Sensor_Connection(i))
            continue;
        uint8_t tx[2] = {0x00, 0x00};
        uint8_t rx[2] = {0};
        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(sensors[i].spi, tx, rx, 2, 100);
        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
        Debug_Print(LOG_LEVEL_INFO, "SPI test sensor %d: NOP resp 0x%02X 0x%02X",
                    i, rx[0], rx[1]);
    }
}

void Measure_SPI_Timing(void)
{
    // Заглушка: можно позже измерить реальное время
    Debug_Print(LOG_LEVEL_INFO, "MeasureSPITiming: stub");
}

uint8_t Verify_SPI_Communication(void)
{
    // Заглушка: считаем, что SPI жив, если хотя бы один сенсор подключен
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (Test_Sensor_Connection(i))
            return 1;
    }
    return 0;
}

// Публичная функция для чтения регистра
uint8_t MLX90393_Read_Register(uint8_t sensor_idx, uint8_t addr, uint16_t *value) {
    if (sensor_idx >= NUM_SENSORS) return 0;
    return mlx90393_read_reg(&sensors[sensor_idx], addr, value);
}

// Публичная функция для сброса датчика
void MLX90393_Reset_Sensor(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return;
    MLX90393_t *sensor = &sensors[sensor_idx];

    // Hard reset через CS
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(100);

    // Программный сброс
    mlx90393_reset(sensor);
    HAL_Delay(5);

    // Выход из всех режимов
    mlx90393_exit(sensor);
    HAL_Delay(5);

    // Переконфигурация
    ConfigureSensor(sensor_idx);
}

// Публичная функция для получения сырых данных
uint8_t MLX90393_Get_Raw_Data(uint8_t sensor_idx, int16_t *t, int16_t *x, int16_t *y, int16_t *z) {
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *sensor = &sensors[sensor_idx];

    if (!sensor->is_connected) return 0;

    uint8_t status = 0;
    if (!mlx90393_start_sm_xyzT(sensor, &status)) return 0;

    HAL_Delay(10); // Увеличьте задержку для измерений

    if (!mlx90393_read_rm_xyzT(sensor, t, x, y, z)) return 0;

    return 1;
}
