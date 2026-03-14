#include "sensor_mlx90393.h"
#include "config.h"
#include "qspi_flash.h"
#include "debug_console.h"
#include "spi.h"
#include <string.h>
#include <stdio.h>   // для snprintf
#include <math.h>
#include "main.h" // может понадобиться для HAL_GPIO_WritePin и т.д.
#include "core_cm7.h"  // для функций работы с кэшем
extern SPI_HandleTypeDef hspi1;

// ==================== Задержки ====================
#define CS_DELAY_MS         0
#define POST_CS_DELAY_MS    0
#define EXIT_DELAY_MS       0
#define SM_CONV_DELAY_MS    1

// ==================== Глобальный массив датчиков ====================
MLX90393_t sensors[NUM_SENSORS];

uint8_t current_dma_sensor = 0; // индекс датчика, для которого запущена текущая DMA-передача
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

static uint8_t dma_state_machine = 0;        // 0 - idle, 1 - ожидание преобразования, 2 - чтение
static uint32_t conversion_start_time = 0;
static const uint8_t start_cmd[2] = {0x3F, 0x00}; // команда запуска измерения

// ==================== Внутренние функции (определения) ====================
static void Start_All_Sensors_Conversion(void);
static void Start_Next_Sensor_Read(void);

static uint8_t mlx90393_reset(MLX90393_t *sensor) {
    uint8_t tx[2] = {0xF0, 0x00};
    uint8_t rx[2];
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK) {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(EXIT_DELAY_MS);
    return 1;
}

static uint8_t mlx90393_exit(MLX90393_t *sensor) {
    uint8_t tx[2] = {0x80, 0x00};
    uint8_t rx[2];
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK) {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(EXIT_DELAY_MS);
    return 1;
}

static uint8_t mlx90393_start_sm_xyzT(MLX90393_t *sensor, uint8_t *status) {
    uint8_t tx[2] = {0x3F, 0x00};
    uint8_t rx[2];
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 2, 100) != HAL_OK) {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(POST_CS_DELAY_MS);
    if (status) *status = rx[0];
    return 1;   // игнорируем ERROR бит
}

static uint8_t mlx90393_read_rm_xyzT(MLX90393_t *sensor,
                                      int16_t *xraw, int16_t *yraw, int16_t *zraw,
                                      uint16_t *traw) {
    uint8_t tx[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
    uint8_t rx[10];
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    if (HAL_SPI_TransmitReceive(sensor->spi, tx, rx, 10, 100) != HAL_OK) {
        HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);
        return 0;
    }
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    HAL_Delay(POST_CS_DELAY_MS);

    if (traw) *traw = (uint16_t)((rx[2] << 8) | rx[3]);
    if (xraw) *xraw = (int16_t)((rx[4] << 8) | rx[5]);
    if (yraw) *yraw = (int16_t)((rx[6] << 8) | rx[7]);
    if (zraw) *zraw = (int16_t)((rx[8] << 8) | rx[9]);
    return 1;
}

// ==================== Вспомогательная функция для подсчёта контрольной суммы ====================
static uint32_t calculate_calibration_checksum(const CalibrationData_t *data) {
    uint32_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t size = sizeof(CalibrationData_t) - sizeof(data->checksum);
    for (size_t i = 0; i < size; i++) sum += bytes[i];
    return sum;
}

// ==================== Конфигурация датчика ====================
void ConfigureSensor(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return;
    MLX90393_t *s = &sensors[sensor_idx];

    mlx90393_exit(s);
    mlx90393_reset(s);
    mlx90393_exit(s);
    HAL_Delay(20);

    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0xFF, 0xFF};
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(CS_DELAY_MS);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(s->spi, tx, rx, 2, 100);
    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_SET);
    HAL_Delay(POST_CS_DELAY_MS);

    if (status == HAL_OK && !(rx[0] == 0xFF && rx[1] == 0xFF)) {
        s->is_connected = 1;
        Debug_Print(LOG_LEVEL_INFO, "Sensor %d ready (factory settings, GAIN_SEL=7)\r\n", sensor_idx);
        s->gain_sel = 1;
    } else {
        s->is_connected = 0;
        Debug_Print(LOG_LEVEL_ERROR, "Sensor %d not responding\r\n", sensor_idx);
    }
}

// ==================== Инициализация всех датчиков ====================
void Sensors_Init(void) {
    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");
    HAL_SPI_DMAStop(&hspi1);
    HAL_Delay(10);
    // Сброс состояний датчиков
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].dma_state = 0;
        sensors[i].is_connected = 0;
    }
    dma_state_machine = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
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
            default: sensors[i].cs_pin = 0; break;
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
        sensors[i].gain_sel         = 7;
        sensors[i].offset[0] = sensors[i].offset[1] = sensors[i].offset[2] = 0.0f;
        sensors[i].scale[0] = sensors[i].scale[1] = sensors[i].scale[2] = 1.0f;
        sensors[i].dma_state = 0;

        memset(sensors[i].tx_dma, 0, sizeof(sensors[i].tx_dma));
        memset(sensors[i].rx_dma, 0, sizeof(sensors[i].rx_dma));
    }

    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++) {
        uint8_t tx[2] = {0x00, 0x00};
        uint8_t rx[2] = {0xFF, 0xFF};

        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_RESET);
        HAL_Delay(CS_DELAY_MS);
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(sensors[i].spi, tx, rx, 2, 100);
        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
        HAL_Delay(POST_CS_DELAY_MS);

        if (status == HAL_OK && !(rx[0] == 0xFF && rx[1] == 0xFF)) {
            Debug_Print(LOG_LEVEL_INFO, "Sensor %d detected\r\n", i);
            ConfigureSensor(i);
            if (sensors[i].is_connected) {
                HAL_Delay(10); // дать время датчику устаканиться
            }
        } else {
            sensors[i].is_connected = 0;
            Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not detected\r\n", i);
        }
    }
    Debug_Print(LOG_LEVEL_INFO, "Connected sensors: ");
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        Debug_Print(LOG_LEVEL_INFO, "%d:%d ", i, sensors[i].is_connected);
    }
    Debug_Print(LOG_LEVEL_INFO, "\n");
    Load_Calibration_From_Flash();
    // Принудительный сброс состояний DMA
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        sensors[i].dma_state = 0;
        sensors[i].dma_start_tick = 0;
    }
    dma_state_machine = 0;
    conversion_start_time = 0;

    // Небольшая задержка перед запуском
    HAL_Delay(100);

    // Принудительно запустить первый цикл DMA
    Update_Sensors_DMA();

}

// ==================== Таблица коэффициентов ====================
static const float gain_coeff[8][2] = {
    {0.751f, 1.210f}, {0.601f, 0.968f}, {0.451f, 0.726f}, {0.376f, 0.605f},
    {0.300f, 0.484f}, {0.250f, 0.403f}, {0.200f, 0.323f}, {0.150f, 0.242f}
};

// ==================== Чтение с учётом GAIN_SEL ====================
uint8_t Read_Sensor_With_Gain(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_connected) return 0;

    uint32_t t0 = HAL_GetTick();

    mlx90393_exit(s);
    HAL_Delay(2);

    uint8_t status;
    if (!mlx90393_start_sm_xyzT(s, &status)) {
        s->failed_reads++;
        s->read_error_count++;
        return 0;
    }

    HAL_Delay(SM_CONV_DELAY_MS);

    int16_t xraw, yraw, zraw;
    uint16_t traw;
    if (!mlx90393_read_rm_xyzT(s, &xraw, &yraw, &zraw, &traw)) {
        s->failed_reads++;
        s->read_error_count++;
        return 0;
    }

    uint8_t gain = (s->gain_sel <= 7) ? s->gain_sel : 7;
    float lsb_xy = gain_coeff[gain][0];
    float lsb_z  = gain_coeff[gain][1];

    s->magnetic_field[0] = xraw * lsb_xy;
    s->magnetic_field[1] = yraw * lsb_xy;
    s->magnetic_field[2] = zraw * lsb_z;
    s->temperature = ((float)traw - 46244.0f) / 45.2f + 25.0f;

    s->total_reads++;
    s->last_read_time = HAL_GetTick();
    float dt = (float)(s->last_read_time - t0);
    if (s->total_reads == 1)
        s->average_read_time_ms = dt;
    else
        s->average_read_time_ms = s->average_read_time_ms * 0.9f + dt * 0.1f;

    return 1;
}

// ==================== Калибровка смещения ====================
void Calibrate_Offset_Procedure(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_connected) {
        Debug_Print(LOG_LEVEL_ERROR, "Sensor %d not connected", sensor_idx);
        return;
    }

    Debug_Print(LOG_LEVEL_INFO, "=== CALIBRATION PROCEDURE Sensor %d ===", sensor_idx);
    Debug_Print(LOG_LEVEL_INFO, "Remove all magnets, wait 3 seconds...");
    HAL_Delay(3000);

    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 20;

    for (int i = 0; i < samples; i++) {
        if (Read_Sensor_With_Gain(sensor_idx)) {
            sum_x += s->magnetic_field[0];
            sum_y += s->magnetic_field[1];
            sum_z += s->magnetic_field[2];
            Debug_Print(LOG_LEVEL_INFO, "Sample %d: X=%.1f, Y=%.1f, Z=%.1f",
                        i, s->magnetic_field[0], s->magnetic_field[1], s->magnetic_field[2]);
        }
        HAL_Delay(100);
    }

    s->offset[0] = -sum_x / samples;
    s->offset[1] = -sum_y / samples;
    s->offset[2] = -sum_z / samples;
    s->is_calibrated = 1;

    Debug_Print(LOG_LEVEL_INFO, "Offset calculated: X=%.1f, Y=%.1f, Z=%.1f µT",
                s->offset[0], s->offset[1], s->offset[2]);

    Read_Sensor_With_Gain(sensor_idx);
    float cal_x = s->magnetic_field[0] + s->offset[0];
    float cal_y = s->magnetic_field[1] + s->offset[1];
    float cal_z = s->magnetic_field[2] + s->offset[2];
    float mag = sqrtf(cal_x*cal_x + cal_y*cal_y + cal_z*cal_z);
    Debug_Print(LOG_LEVEL_INFO, "After offset: X=%.1f, Y=%.1f, Z=%.1f µT, magnitude=%.1f µT",
                cal_x, cal_y, cal_z, mag);
}

// ==================== Чтение с калибровкой ====================
uint8_t Read_Sensor_Calibrated(uint8_t sensor_idx) {
    if (!Read_Sensor_With_Gain(sensor_idx)) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_calibrated) {
        Debug_Print(LOG_LEVEL_INFO, "Sensor %d not calibrated, raw data displayed.", sensor_idx);
        return 1;
    }
    float cal_x = s->magnetic_field[0] + s->offset[0];
    float cal_y = s->magnetic_field[1] + s->offset[1];
    float cal_z = s->magnetic_field[2] + s->offset[2];
    float mag = sqrtf(cal_x*cal_x + cal_y*cal_y + cal_z*cal_z);
    Debug_Print(LOG_LEVEL_INFO, "Sensor %d calibrated: X=%.1f, Y=%.1f, Z=%.1f µT, magnitude=%.1f µT",
                sensor_idx, cal_x, cal_y, cal_z, mag);
    return 1;
}

// ==================== Функции, ожидаемые в main.c ====================
uint8_t Quick_Read_Sensor(uint8_t sensor_idx) {
    return Read_Sensor_Calibrated(sensor_idx);
}

uint8_t Read_Sensor(uint8_t sensor_idx) {
    return Read_Sensor_With_Gain(sensor_idx);
}

void Calibrate_Sensors_Start(void) {
    Debug_Print(LOG_LEVEL_INFO, "Starting calibration of all active sensors...");
    for (uint8_t i = 0; i < ACTIVE_SENSORS; i++) {
        if (sensors[i].is_connected) {
            Calibrate_Offset_Procedure(i);
        }
    }
}

uint8_t Test_Sensor_Connection(uint8_t sensor_idx) {
    if (sensor_idx >= NUM_SENSORS) return 0;
    return sensors[sensor_idx].is_connected;
}
/**
  * @brief  Вычисляет модуль вектора магнитного поля по данным датчика.
  * @param  sensor_idx Индекс датчика (0..NUM_SENSORS-1).
  * @retval Модуль поля в микротеслах (µT) или 0, если датчик недоступен.
  */
float Calculate_Magnetic_Field_Strength(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0.0f;
    MLX90393_t *s = &sensors[sensor_idx];
    float x = s->magnetic_field[0];
    float y = s->magnetic_field[1];
    float z = s->magnetic_field[2];
    return sqrtf(x*x + y*y + z*z);
}

void Get_Sensor_Stats_String(char *buffer, uint16_t buffer_size) {
    int connected = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
        if (sensors[i].is_connected) connected++;
    snprintf(buffer, buffer_size, "Sensors: %d/%d connected", connected, NUM_SENSORS);
}

// ==================== Сохранение/загрузка калибровки ====================
void Save_Calibration_To_Flash(void) {
    Debug_Print(LOG_LEVEL_INFO, "Saving calibration to flash...");
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
    Debug_Print(LOG_LEVEL_INFO, "Calibration saved at 0x%08lX", CALIBRATION_DATA_ADDR);
}

void Load_Calibration_From_Flash(void) {
    Debug_Print(LOG_LEVEL_INFO, "Loading calibration from flash...");
    CalibrationData_t cal_data;
    QSPI_Flash_ReadBuffer(CALIBRATION_DATA_ADDR, (uint8_t*)&cal_data, sizeof(CalibrationData_t));

    if (cal_data.signature != CALIBRATION_DATA_SIGNATURE) {
        Debug_Print(LOG_LEVEL_WARNING, "Invalid signature, skipping load");
        return;
    }
    if (cal_data.version != CALIBRATION_DATA_VERSION) {
        Debug_Print(LOG_LEVEL_WARNING, "Version mismatch, skipping load");
        return;
    }
    uint32_t expected = calculate_calibration_checksum(&cal_data);
    if (cal_data.checksum != expected) {
        Debug_Print(LOG_LEVEL_WARNING, "Checksum mismatch, skipping load");
        return;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < 3; j++) {
            sensors[i].offset[j] = cal_data.offsets[i][j];
            sensors[i].scale[j] = cal_data.scales[i][j];
        }
        sensors[i].is_calibrated = 1;
    }
    Debug_Print(LOG_LEVEL_INFO, "Calibration loaded.");
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        Debug_Print(LOG_LEVEL_ERROR, "SPI error on sensor %d, error=%lu\n", current_dma_sensor, hspi->ErrorCode);
        sensors[current_dma_sensor].dma_state = 0;
        HAL_GPIO_WritePin(sensors[current_dma_sensor].cs_port, sensors[current_dma_sensor].cs_pin, GPIO_PIN_SET);
        HAL_SPI_DMAStop(&hspi1);
    }
}

uint8_t Read_Sensor_DMA_Start(uint8_t sensor_idx)
{
   // Debug_Print(LOG_LEVEL_INFO, "RStart %d\n", sensor_idx);
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (!s->is_connected) {
        Debug_Print(LOG_LEVEL_WARNING, "Sensor %d not connected\n", sensor_idx);
        return 0;
    }
    if (s->dma_state != 0) {
        Debug_Print(LOG_LEVEL_WARNING, "Sensor %d state %d\n", sensor_idx, s->dma_state);
        return 0;
    }

    // Подготовка команды чтения
    uint8_t cmd[10] = {0x4F, 0x00, 0,0,0,0,0,0,0,0};
    memcpy(s->tx_dma, cmd, 10);

    // Очищаем кэш для TX-буфера (чтобы данные точно попали в память)
    SCB_CleanDCache_by_Addr((uint32_t*)s->tx_dma, 10);
    // Инвалидируем RX-буфер перед приёмом (чтобы процессор не использовал старые данные)
    SCB_InvalidateDCache_by_Addr((uint32_t*)s->rx_dma, 10);

    HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 10; i++); // небольшая задержка

    s->dma_state = 1;
    s->dma_start_tick = HAL_GetTick();  // запоминаем время старта
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(s->spi, s->tx_dma, s->rx_dma, 10);
    if (status != HAL_OK) {
        Debug_Print(LOG_LEVEL_ERROR, "DMA start failed for sensor %d, status=%d\n", sensor_idx, status);
        HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_SET);
        s->dma_state = 0;
        return 0;

    }
    return 1;
}

uint8_t Process_Sensor_DMA_Data(uint8_t sensor_idx)
{
    if (sensor_idx >= NUM_SENSORS) return 0;
    MLX90393_t *s = &sensors[sensor_idx];
    if (s->dma_state != 2) return 0;

    // Парсинг данных из rx_dma
    uint16_t traw = (s->rx_dma[2] << 8) | s->rx_dma[3];
    int16_t xraw = (int16_t)((s->rx_dma[4] << 8) | s->rx_dma[5]);
    int16_t yraw = (int16_t)((s->rx_dma[6] << 8) | s->rx_dma[7]);
    int16_t zraw = (int16_t)((s->rx_dma[8] << 8) | s->rx_dma[9]);

    uint8_t gain = (s->gain_sel <= 7) ? s->gain_sel : 7;
    float lsb_xy = gain_coeff[gain][0];
    float lsb_z  = gain_coeff[gain][1];

    // Защита от прерываний при обновлении данных
    __disable_irq();
    s->magnetic_field[0] = xraw * lsb_xy;
    s->magnetic_field[1] = yraw * lsb_xy;
    s->magnetic_field[2] = zraw * lsb_z;
    s->temperature = ((float)traw - 46244.0f) / 45.2f + 25.0f;
    __enable_irq();

    s->dma_state = 0; // теперь датчик свободен
    s->last_read_time = HAL_GetTick();
    return 1;
}

void Update_Sensors_DMA(void)
{
	static uint32_t last_dma_log = 0;
	    uint32_t now = HAL_GetTick();

	    // Выводим состояние раз в 2 секунды, чтобы не засорять лог
	    if (now - last_dma_log > 2000) {
	        last_dma_log = now;
	        Debug_Print(LOG_LEVEL_INFO, "DMA state: %d, time since conv: %lu ms\n",
	                    dma_state_machine, now - conversion_start_time);
	    }

    switch (dma_state_machine) {
        case 0:
        {
            static uint32_t last_conv_start = 0;
            // Не запускать новое преобразование слишком часто
            if (now - last_conv_start < 10) {
                break;
            }
            uint8_t all_free = 1;
            for (int i = 0; i < ACTIVE_SENSORS; i++) {
                if (sensors[i].dma_state != 0) {
                    all_free = 0;
                    break;
                }
            }
            if (all_free) {
                Start_All_Sensors_Conversion();
                last_conv_start = now;
                conversion_start_time = now;
                dma_state_machine = 1;
            }
            break;
        }
        case 1:
        {
            if (now - conversion_start_time >= 2) {
                dma_state_machine = 2;
                Start_Next_Sensor_Read();
            }
            break;
        }
        case 2:
        {
            // Обработка готовых данных
            for (int i = 0; i < ACTIVE_SENSORS; i++) {
                if (sensors[i].dma_state == 2) {
                    Process_Sensor_DMA_Data(i);
                }
                if (sensors[i].dma_state == 1) {
                    if (now - sensors[i].dma_start_tick > 200) {
                        Debug_Print(LOG_LEVEL_ERROR, "DMA timeout for sensor %d, SPI state=%d, error=%lu\n",
                                    i, hspi1.State, hspi1.ErrorCode);
                        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
                        HAL_SPI_DMAStop(&hspi1);
                        sensors[i].dma_state = 0;
                    }
                }
            }

            // Проверка таймаута
            for (int i = 0; i < ACTIVE_SENSORS; i++) {
                if (sensors[i].dma_state == 1) {
                    if (now - sensors[i].dma_start_tick > 200) {
                        Debug_Print(LOG_LEVEL_ERROR, "DMA timeout for sensor %d\n", i);
                        HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
                        HAL_SPI_DMAStop(&hspi1);
                        sensors[i].dma_state = 0;
                    }
                }
            }

            uint8_t busy = 0;
            for (int i = 0; i < ACTIVE_SENSORS; i++) {
                if (sensors[i].dma_state == 1) {
                    busy = 1;
                    break;
                }
            }

            if (!busy) {
                uint8_t all_done = 1;
                for (int i = 0; i < ACTIVE_SENSORS; i++) {
                    if (sensors[i].dma_state != 0) {
                        all_done = 0;
                        break;
                    }
                }
                if (all_done) {
                    dma_state_machine = 0;
                } else {
                    Start_Next_Sensor_Read();
                }
            }
            break;
        }
    }
    if (dma_state_machine == 1 && now - conversion_start_time > 500) {
        Debug_Print(LOG_LEVEL_WARNING, "DMA machine stalled, resetting\n");
        dma_state_machine = 0;
        for (int i = 0; i < ACTIVE_SENSORS; i++) {
            sensors[i].dma_state = 0;
        }
    }
}

static void Start_All_Sensors_Conversion(void)
{
	// Debug_Print(LOG_LEVEL_INFO, "Start conversion\n");
    // Опускаем CS для всех подключенных датчиков
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (sensors[i].is_connected) {
            HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_RESET);
        }
    }
    // Небольшая задержка для установки CS
    for (volatile int i = 0; i < 10; i++);
    HAL_SPI_DMAStop(&hspi1);

    // Отправляем команду (блокирующе)
    HAL_SPI_Transmit(&hspi1, (uint8_t*)start_cmd, 2, 10);

    // Поднимаем CS для всех датчиков
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (sensors[i].is_connected) {
            HAL_GPIO_WritePin(sensors[i].cs_port, sensors[i].cs_pin, GPIO_PIN_SET);
        }
    }
}

static void Start_Next_Sensor_Read(void)
{
    static uint8_t next_sensor = 0;
    for (int attempt = 0; attempt < ACTIVE_SENSORS; attempt++) {
        uint8_t idx = next_sensor;
        next_sensor = (next_sensor + 1) % ACTIVE_SENSORS;
        if (sensors[idx].is_connected && sensors[idx].dma_state == 0) {
            current_dma_sensor = idx;
            if (Read_Sensor_DMA_Start(idx)) {
                break;
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Failed to start read for %d\n", idx);
            }
        }
    }
}
/**
  * @brief  Callback при завершении передачи по SPI с DMA.
  * @param  hspi: указатель на структуру SPI_HandleTypeDef
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        // Проверка на ошибки SPI
        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
            Debug_Print(LOG_LEVEL_ERROR, "SPI error: %lu\n", hspi->ErrorCode);
        }

        // Получаем указатель на текущий датчик (глобальная переменная current_dma_sensor)
        MLX90393_t *s = &sensors[current_dma_sensor];

        // Инвалидируем кэш для приёмного буфера, чтобы процессор увидел свежие данные
        SCB_InvalidateDCache_by_Addr((uint32_t*)s->rx_dma, 10);

        // Поднимаем CS
        HAL_GPIO_WritePin(s->cs_port, s->cs_pin, GPIO_PIN_SET);

        // Останавливаем DMA (на всякий случай)
        HAL_SPI_DMAStop(hspi);

        // Отмечаем, что данные готовы
        s->dma_state = 2;

        // Опционально: отладочный вывод (можно закомментировать)
        // Debug_Print(LOG_LEVEL_INFO, "DMA complete for sensor %d\n", current_dma_sensor);
    }
}
