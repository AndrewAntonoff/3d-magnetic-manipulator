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
#include "main.h"
#include "dma.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "config.h"
#include "debug_console.h"
#include "coil_driver.h"
#include "levitation_control.h"
#include "sensor_mlx90393.h"
#include "qspi_flash.h"
#include "coil_calib.h"
#include "vl53l5x_interface.h"
#include "i2c.h"

#define BINARY_CMD "b_all"
#define SENSOR_FLOATS 15
#define COIL_FLOATS 12

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    WAIT_FOR_AA,
    WAIT_FOR_55,
    RECEIVING_DATA
} ImuRxState_t;

static uint8_t binary_rx_mode = 0;
static uint8_t binary_rx_buffer[sizeof(CoilCalibData_t)];
static uint32_t binary_rx_index = 0;
static uint32_t binary_rx_total = 0;

static uint8_t coil_rx_mode = 0;                // флаг приёма данных катушек
static uint8_t coil_rx_buffer[48];               // буфер для 12 float (48 байт)
static uint32_t coil_rx_index = 0;                // текущий индекс в буфере
static uint32_t coil_rx_total = 48;                // ожидаемое количество байт
static uint32_t coil_rx_last_byte = 0;             // время последнего принятого байта
// static uint32_t coil_rx_start = 0;                  // время начала режима
static uint8_t echo_enabled = 1;   // разрешить эхо команд
volatile uint8_t training_mode = 0;   // 0 – обычный режим, 1 – режим обучения


static ImuRxState_t imu_rx_state = WAIT_FOR_AA;
static uint8_t imu_temp_buf[IMU_PACKET_SIZE];
static uint8_t imu_temp_idx;
volatile uint8_t control_tick = 0;

// Глобальные переменные для IMU
volatile IMU_Data_t last_imu_data;
volatile uint8_t imu_packet_ready = 0;

// Буферы
char console_buffer[256];
uint8_t command_buffer[64];
uint16_t command_index = 0;
uint8_t RxChar;

// Тайминги
uint32_t last_monitor_time = 0;
uint32_t last_cpu_measure = 0;
uint32_t last_stream_time = 0;

// Статистика
uint32_t total_commands = 0;
uint32_t idle_counter = 0;

// Флаги
uint8_t new_command = 0;
uint8_t streaming_active = 0;
uint32_t stream_interval_ms = 50;

static volatile uint8_t binary_mode = 0;


// Глобальные переменные отчета HID (Dual-Mode)
// --- USB Dual-Mode Variables ---
__ALIGN_BEGIN static uint8_t Joystick_ReportDesc[65] __ALIGN_END = {
    0x05, 0x01, 0x09, 0x04, 0xA1, 0x01, 0x85, 0x01,
    0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33,
    0x09, 0x34, 0x09, 0x35, 0x16, 0x01, 0x80, 0x26,
    0xFF, 0x7F, 0x75, 0x10, 0x95, 0x06, 0x81, 0x02,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x20, 0x15, 0x00,
    0x25, 0x01, 0x75, 0x01, 0x95, 0x20, 0x81, 0x02,
    /* Feature Report 32-bytes ID 0x10 */
    0x85, 0x10, 0x09, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x20, 0xB1, 0x02,
    0xC0
};

__ALIGN_BEGIN static uint8_t SpaceMouse_ReportDesc[98] __ALIGN_END = {
    0x05, 0x01, 0x09, 0x08, 0xA1, 0x01, 
    0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x36, 0x00, 0x80, 0x46, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02,
    0x85, 0x02, 0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F, 0x36, 0x00, 0x80, 0x46, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02,
    0x85, 0x03, 0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02, 0x95, 0x01, 0x75, 0x06, 0x81, 0x03,
    /* Feature Report 32-bytes ID 0x10 */
    0x85, 0x10, 0x09, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x20, 0xB1, 0x02,
    0xC0
};

uint8_t *Active_HID_ReportDesc = SpaceMouse_ReportDesc; 
uint16_t Active_HID_ReportDescSize = sizeof(SpaceMouse_ReportDesc);
uint8_t current_usb_mode = 1; // 1 = SpaceMouse, 0 = Joystick

/* USER CODE END PTD */

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
#define CMD_TEST_COIL      "test_coil"
#define CMD_TEST_ALL       "test_all"
#define CMD_TEST_GROUP     "test_group"
#define CMD_STOP_TEST      "stop_test"
#define CMD_COILS_OFF      "coils_off"
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
void Switch_USB_Mode(uint8_t mode);
// Прототипы внутренних функций
void System_Init(void);
void Process_Console_Commands(void);
void Update_System_Status(void);
void Calculate_CPU_Usage(void);
void Update_Uptime(void);
void Show_Help_Menu(void);
void Show_System_Status(void);
void Stream_Sensor_Data(void);
void Error_Handler(void);
void Process_IMU_Byte(uint8_t byte);
uint8_t Read_Sensor_With_Gain(uint8_t sensor_idx);
void Calibrate_Offset_Procedure(uint8_t sensor_idx);
uint8_t Read_Sensor_Calibrated(uint8_t sensor_idx);
void Save_Calibration_To_Flash(void);
void Load_Calibration_From_Flash(void);
uint8_t Quick_Read_Sensor(uint8_t sensor_idx);
uint8_t Read_Sensor(uint8_t sensor_idx);
void Calibrate_Sensors_Start(void);
void Get_Sensor_Stats_String(char *buffer, uint16_t buffer_size);
/* Receive_Coil_Calibration() removed — dead code, binary rx handled via binary_rx_mode flag */
void Process_Coil_Calib_Data(uint8_t *buffer, uint32_t len);
uint8_t Test_Sensor_Connection(uint8_t sensor_idx);
void Check_QSPI(void);

// Новая функция для управления левитацией
void SystemControlLoop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Инициализация системы
void System_Init(void) {
    Debug_Init(&huart2);
    Debug_Print(LOG_LEVEL_INFO, "=== Magnetic Manipulator System Initialization ===\r\n");

    Coils_Init();
    system_state.coils_enabled = 1;
    Initialize_Coil_Geometry();
    Initialize_Sensor_Geometry();
    Load_Coil_Calibration();

    // Проверка калибровки катушек
    float test_field[8][3];
    Get_Coil_Field(0, 0.0f, test_field);
    Debug_Print(LOG_LEVEL_INFO, "Coil 0 at I=0: sensor0 field = %.1f, %.1f, %.1f µT\r\n",
                test_field[0][0], test_field[0][1], test_field[0][2]);

    Get_Coil_Field(0, 1.0f, test_field);
    Debug_Print(LOG_LEVEL_INFO, "Coil 0 at I=1: sensor0 field = %.1f, %.1f, %.1f µT\r\n",
                test_field[0][0], test_field[0][1], test_field[0][2]);

    Debug_Print(LOG_LEVEL_INFO, "Initializing magnetic sensors...\r\n");
    Sensors_Init();
    system_state.sensors_enabled = 1;

    if (!QSPI_Flash_Init()) {
        Debug_Print(LOG_LEVEL_ERROR, "Failed to initialize QSPI Flash!\r\n");
    } else {
        Debug_Print(LOG_LEVEL_INFO, "QSPI Flash initialized.\r\n");
        Check_QSPI();
    }
    HAL_Delay(100);
    Debug_Print(LOG_LEVEL_INFO, "Loading calibration data...\r\n");
    Load_Calibration_From_Flash();

    Debug_Print(LOG_LEVEL_INFO, "System initialization complete.\r\n");
    VL53L5X_Init();
}

// Обработка байта IMU
void Process_IMU_Byte(uint8_t byte) {
    switch (imu_rx_state) {
        case WAIT_FOR_AA:
            if (byte == 0xAA) imu_rx_state = WAIT_FOR_55;
            break;
        case WAIT_FOR_55:
            if (byte == 0x55) {
                imu_rx_state = RECEIVING_DATA;
                imu_temp_idx = 0;
            } else {
                imu_rx_state = WAIT_FOR_AA;
            }
            break;
        case RECEIVING_DATA:
            imu_temp_buf[imu_temp_idx++] = byte;
            if (imu_temp_idx >= IMU_PACKET_SIZE) {
                memcpy((void*)&last_imu_data, imu_temp_buf, IMU_PACKET_SIZE);
                imu_packet_ready = 1;
                imu_rx_state = WAIT_FOR_AA;
            }
            break;
    }
}

void Update_System_Status(void) {
    Update_Uptime();
    Calculate_CPU_Usage();
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
    if(elapsed >= 1000) {
        float usage = 0.0f; // Заглушка
        system_state.cpu_usage_percent = usage;
        idle_counter = 0;
        last_cpu_measure = current_time;
    }
}

void Update_Uptime(void) {
    system_state.system_uptime_ms = HAL_GetTick();
}

void Show_Help_Menu(void) {
    Debug_Print(LOG_LEVEL_INFO, "Available commands:\r\n");
    Debug_Print(LOG_LEVEL_INFO, "help - Show this menu\r\n");
    Debug_Print(LOG_LEVEL_INFO, "status - Show system status\r\n");
    Debug_Print(LOG_LEVEL_INFO, "sensor <idx> - Read sensor data\r\n");
    Debug_Print(LOG_LEVEL_INFO, "calibrate [idx] - Calibrate sensor offset (all sensors if no index)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "save_cal - Save calibration data to flash\r\n");
    Debug_Print(LOG_LEVEL_INFO, "load_cal - Load calibration data from flash\r\n");
    Debug_Print(LOG_LEVEL_INFO, "start_stream [interval_ms] - Start data streaming (default: 50 ms)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stop_stream - Stop data streaming\r\n");
    Debug_Print(LOG_LEVEL_INFO, "levitate - Start levitation control\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stop_levitate - Stop levitation control\r\n");
    Debug_Print(LOG_LEVEL_INFO, "set_target x y z - Set levitation target position\r\n");
    Debug_Print(LOG_LEVEL_INFO, "read_cal <idx> - Read sensor with calibration applied\r\n");
    Debug_Print(LOG_LEVEL_INFO, "read_raw <idx> - Read raw sensor data (with gain)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "test_coil <idx> <start> <end> <step> <duration_ms> - Run test on single coil\r\n");
    Debug_Print(LOG_LEVEL_INFO, "test_all <power> - Set all coils to power (-1..1)\r\n");
    Debug_Print(LOG_LEVEL_INFO, "test_group <mask> <power> - Set coils by bit mask (hex) to power\r\n");
    Debug_Print(LOG_LEVEL_INFO, "stop_test - Stop current coil test\r\n");
    Debug_Print(LOG_LEVEL_INFO, "coils_off - Turn off all coils\r\n");
}

void Show_Prompt(void) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n> ", 4, 10);
}

void Show_System_Status(void) {
    Debug_Print(LOG_LEVEL_INFO, "System status:\r\n");
    Debug_Print(LOG_LEVEL_INFO, "Uptime: %lu ms\r\n", system_state.system_uptime_ms);
    Debug_Print(LOG_LEVEL_INFO, "CPU usage: %.1f%%\r\n", system_state.cpu_usage_percent);
    Debug_Print(LOG_LEVEL_INFO, "Coils enabled: %d\r\n", system_state.coils_enabled);
    Debug_Print(LOG_LEVEL_INFO, "Sensors enabled: %d\r\n", system_state.sensors_enabled);
    Debug_Print(LOG_LEVEL_INFO, "Calibration done: %d\r\n", system_state.calibration_done);
    Debug_Print(LOG_LEVEL_INFO, "Monitoring active: %d\r\n", system_state.monitoring_active);
    Debug_Print(LOG_LEVEL_INFO, "Levitation active: %d\r\n", system_state.levitation_active);
    Debug_Print(LOG_LEVEL_INFO, "Ball position: X=%.2f, Y=%.2f, Z=%.2f\r\n",
                system_state.ball_position[0], system_state.ball_position[1], system_state.ball_position[2]);

    Get_Sensor_Stats_String(console_buffer, sizeof(console_buffer));
    Debug_Print(LOG_LEVEL_INFO, "%s\r\n", console_buffer);
}

void Stream_Sensor_Data(void) {
    if (!streaming_active) return;
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_stream_time < stream_interval_ms) {
        return;
    }
    last_stream_time = current_time;

    char buffer[128];
    int len = snprintf(buffer, sizeof(buffer), "%lu", current_time);

    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (sensors[i].is_connected) {
            // Apply full calibration (offset + scale) for streamed data
            float x = (sensors[i].magnetic_field[0] + sensors[i].offset[0]) * sensors[i].scale[0];
            float y = (sensors[i].magnetic_field[1] + sensors[i].offset[1]) * sensors[i].scale[1];
            float z = (sensors[i].magnetic_field[2] + sensors[i].offset[2]) * sensors[i].scale[2];
            len += snprintf(buffer + len, sizeof(buffer) - len, ",%.1f,%.1f,%.1f", x, y, z);
        } else {
            len += snprintf(buffer + len, sizeof(buffer) - len, ",0,0,0");
        }
    }
    len += snprintf(buffer + len, sizeof(buffer) - len, ",%.1f,%.1f,%.1f",
                    system_state.ball_position[0],
                    system_state.ball_position[1],
                    system_state.ball_position[2]);
    strcat(buffer, "\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 10);
}

void Process_Coil_Calib_Data(uint8_t *buffer, uint32_t len) {
    Debug_Print(LOG_LEVEL_INFO, "Processing %d bytes...\r\n", len);
    if (len != sizeof(CoilCalibData_t)) {
        Debug_Print(LOG_LEVEL_ERROR, "Data size mismatch: expected %d, got %d\r\n",
                    sizeof(CoilCalibData_t), len);
        return;
    }

    CoilCalibHeader_t *hdr = (CoilCalibHeader_t*)buffer;
    Debug_Print(LOG_LEVEL_INFO, "Signature: 0x%08X, Version: %d\r\n",
                hdr->signature, hdr->version);

    if (hdr->signature != COIL_CALIB_SIGNATURE) {
        Debug_Print(LOG_LEVEL_ERROR, "Invalid signature\r\n");
        return;
    }
    if (hdr->version != COIL_CALIB_VERSION) {
        Debug_Print(LOG_LEVEL_ERROR, "Invalid version\r\n");
        return;
    }
    if (hdr->num_coils != COIL_CALIB_NUM_COILS ||
        hdr->num_sensors != COIL_CALIB_NUM_SENSORS ||
        hdr->num_points != COIL_CALIB_NUM_POINTS) {
        Debug_Print(LOG_LEVEL_ERROR, "Size mismatch: coils=%d, sensors=%d, points=%d\r\n",
                    hdr->num_coils, hdr->num_sensors, hdr->num_points);
        return;
    }

    Debug_Print(LOG_LEVEL_INFO, "Erasing sectors...\r\n");
    for (uint32_t addr = COIL_CALIB_FLASH_ADDR; addr < COIL_CALIB_FLASH_ADDR + sizeof(CoilCalibData_t); addr += 4096) {
        QSPI_Flash_EraseSector(addr);
        Debug_Print(LOG_LEVEL_INFO, "Erased sector at 0x%06lX\r\n", addr);
    }

    Debug_Print(LOG_LEVEL_INFO, "Writing data to QSPI...\r\n");
    QSPI_Flash_WriteBuffer(COIL_CALIB_FLASH_ADDR, buffer, sizeof(CoilCalibData_t));
    Debug_Print(LOG_LEVEL_INFO, "Coil calibration saved to QSPI at 0x%06lX\r\n", COIL_CALIB_FLASH_ADDR);
    HAL_UART_Transmit(&huart2, (uint8_t*)"OK\r\n", 4, 100);
}

/* Receive_Coil_Calibration() removed — was dead code (never called).
   Binary reception is handled via the binary_rx_mode flag in HAL_UART_RxCpltCallback.
   Use the 'flash_coil' console command to trigger calibration upload. */

#include "usbd_hid.h"
#include "usbd_desc.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t USBD_FS_DeviceDesc[];

static int16_t hid_axes[6] = {0};
float sensitivity_scales[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

void Process_Feature_Report(uint8_t *data) {
    if (!data) return;
    
    // Windows hidapi sends Report ID as the first byte of payload
    if (data[0] == 0x10) {
        uint8_t cmd = data[1];
        if (cmd == 0x01) { // Switch USB Mode
            uint8_t mode = data[2];
            Switch_USB_Mode(mode);
        }
        else if (cmd == 0x02) { // Set Sensitivity
            memcpy(sensitivity_scales, &data[2], 24); // 6 floats = 24 bytes
        }
        else if (cmd == 0x03) { // Trigger Calibration
            for (int i = 0; i < ACTIVE_SENSORS; i++) {
                if (sensors[i].is_connected) {
                    Calibrate_Offset_Procedure(i);
                }
            }
        }
        else if (cmd == 0x04) { // Save Calibration
            Save_Calibration_To_Flash();
        }
    }
}

void Calculate_3D_Position(void) {
    // Map ball position mm to generic joystick range (-30000 to +30000 approx for +/- 50mm)
    hid_axes[0] = (int16_t)(system_state.ball_position[0] * 600.0f * sensitivity_scales[0]);
    hid_axes[1] = (int16_t)(system_state.ball_position[1] * 600.0f * sensitivity_scales[1]);
    hid_axes[2] = (int16_t)(system_state.ball_position[2] * 600.0f * sensitivity_scales[2]);
    
    // Map IMU orientation rad*1000 into joystick rotation axes
    hid_axes[3] = (int16_t)(last_imu_data.pitch * 10 * sensitivity_scales[3]);
    hid_axes[4] = (int16_t)(last_imu_data.yaw * 10 * sensitivity_scales[4]);
    hid_axes[5] = (int16_t)(last_imu_data.roll * 10 * sensitivity_scales[5]);
}

void Send_HID_Report(void) {
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;

    if (current_usb_mode == 0) {
        // Joystick Mode: Send all 6 axes securely
        uint8_t jbuf[17] = {0};
        jbuf[0] = 1; // Report ID
        memcpy(&jbuf[1], hid_axes, 12);
        USBD_HID_SendReport(&hUsbDeviceFS, jbuf, 17);
    } else {
        // SpaceMouse Mode: Send separate translation or rotation alternating frames
        static uint8_t toggle = 0;
        uint8_t sbuf[7];
        if (toggle == 0) {
            sbuf[0] = 1; // Translation ID
            memcpy(&sbuf[1], &hid_axes[0], 6);
            toggle = 1;
        } else {
            sbuf[0] = 2; // Rotation ID
            memcpy(&sbuf[1], &hid_axes[3], 6);
            toggle = 0;
        }
        USBD_HID_SendReport(&hUsbDeviceFS, sbuf, 7);
    }
}

void Switch_USB_Mode(uint8_t mode) {
    if (current_usb_mode == mode) return;
    current_usb_mode = mode;
    
    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
    HAL_Delay(500); // Trigger host disconnect detection
    
    if (mode == 0) {
        // Generic Joystick STMicro
        USBD_FS_DeviceDesc[8]  = LOBYTE(1155); 
        USBD_FS_DeviceDesc[9]  = HIBYTE(1155);
        USBD_FS_DeviceDesc[10] = LOBYTE(22315);
        USBD_FS_DeviceDesc[11] = HIBYTE(22315);
        Active_HID_ReportDesc = Joystick_ReportDesc;
        Active_HID_ReportDescSize = sizeof(Joystick_ReportDesc);
    } else {
        // SpaceNavigator 3Dconnexion
        USBD_FS_DeviceDesc[8]  = 0x6F; 
        USBD_FS_DeviceDesc[9]  = 0x25;
        USBD_FS_DeviceDesc[10] = 0x26;
        USBD_FS_DeviceDesc[11] = 0xC6;
        Active_HID_ReportDesc = SpaceMouse_ReportDesc;
        Active_HID_ReportDescSize = sizeof(SpaceMouse_ReportDesc);
    }
    
    USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID);
    USBD_Start(&hUsbDeviceFS);
}

// Новая функция управления левитацией
void SystemControlLoop(void)
{
    if (!training_mode) return;

    static uint32_t last_packet_time = 0;
    static uint8_t init_done = 0; // Флаг одиночной инициализации

    // Выполняется ТОЛЬКО ОДИН РАЗ при старте режима левитации/обучения
    if (!init_done) {
        HAL_UART_AbortReceive(&huart2); // Жестко выключаем прерывания от консоли
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        init_done = 1;
        last_packet_time = HAL_GetTick();
    }

    // 1. ПЕРЕДАЧА ДАННЫХ В PYTHON
    float sensor_buffer[15];
    for (int i = 0; i < 5; i++) {
        sensor_buffer[i*3]   = sensors[i].magnetic_field[0];
        sensor_buffer[i*3+1] = sensors[i].magnetic_field[1];
        sensor_buffer[i*3+2] = sensors[i].magnetic_field[2];
    }
    uint32_t tx_marker = 0xDEADBEEF;
    HAL_UART_Transmit(&huart2, (uint8_t*)&tx_marker, 4, 10);
    HAL_UART_Transmit(&huart2, (uint8_t*)sensor_buffer, 60, 20);

    // 2. ПРИЕМ ДАННЫХ ОТ PYTHON
    // Снимаем только флаг переполнения, если он возник из-за наводок
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
    }

    uint32_t rx_marker = 0;
    // Buffer holds COIL_FLOATS (12) floats for protocol compatibility with Python,
    // but only NUM_COILS (8) are applied. Extra values are silently ignored.
    float coil_buffer[COIL_FLOATS] = {0.0f};

    if (HAL_UART_Receive(&huart2, (uint8_t*)&rx_marker, 4, 10) == HAL_OK) {
        if (rx_marker == 0xDEADBEEF) {
            // Маркер совпал, забираем токи
            if (HAL_UART_Receive(&huart2, (uint8_t*)coil_buffer, COIL_FLOATS * sizeof(float), 15) == HAL_OK) {
                last_packet_time = HAL_GetTick(); // Успех! Сбрасываем таймер
                for (int i = 0; i < NUM_COILS; i++) {
                    Set_Coil_Power(i, coil_buffer[i]);
                }
            }
        } else {
            // Вычищаем мусор из буфера, если пришел не маркер
            uint8_t dummy;
            while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
                HAL_UART_Receive(&huart2, &dummy, 1, 1);
            }
        }
    }

    // 3. ЗАЩИТА ОТ ПОТЕРИ СВЯЗИ
    if (HAL_GetTick() - last_packet_time > 1000) {
        training_mode = 0;
        init_done = 0; // Сбрасываем флаг для следующего запуска
        Stop_All_Coils();

        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
        HAL_UART_Receive_IT(&huart2, &RxChar, 1); // Возвращаем текстовую консоль

        Debug_Print(LOG_LEVEL_ERROR, "Link timeout. Coils OFF.\n");
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_QUADSPI_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE BEGIN 2 */

  System_Init();
  HAL_Delay(500); // дать время всей периферии стабилизироваться

  HAL_UART_Receive_IT(&huart2, &RxChar, 1);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); // Включаем прерывание для приема IMU-байт по UART3

  // Запуск ШИМ для всех катушек
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0); // TIM6 – высокий приоритет
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  // Приоритеты DMA (уже должны быть установлены в HAL_SPI_MspInit, но можно переназначить)
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  Stop_All_Coils();

  Debug_Print(LOG_LEVEL_INFO, "\r\n");
  Debug_Print(LOG_LEVEL_INFO, "========================================\r\n");
  Debug_Print(LOG_LEVEL_INFO, "   MAGNETIC MANIPULATOR CONTROL SYSTEM\r\n");
  Debug_Print(LOG_LEVEL_INFO, "   Version: 2.0 Cleaned | Coils: %d | Sensors: %d\r\n", NUM_COILS, NUM_SENSORS);
  Debug_Print(LOG_LEVEL_INFO, "========================================\r\n\r\n");

  Show_Help_Menu();
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n> ", 4, 10);

  last_monitor_time = HAL_GetTick();
  last_cpu_measure = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
      // Управление левитацией (вынесено в отдельную функцию)
      if (control_tick)
      {
          if (training_mode) {
              SystemControlLoop();
          } else if (system_state.levitation_active) {
              Apply_Levitation_Control();
          }
          control_tick = 0;
      }

      // Проверка таймаута приёма данных катушек
      if (coil_rx_mode && (HAL_GetTick() - coil_rx_last_byte > 200)) {
          coil_rx_mode = 0;
          coil_rx_index = 0;
          Debug_Print(LOG_LEVEL_WARNING, "Coil RX timeout, mode reset\r\n");
      }

      // Обработка команд консоли
      Process_Console_Commands();

      // Потоковая передача данных
      Stream_Sensor_Data();

      // Обновление системного статуса
      Update_System_Status();

      // Обработка теста катушек
      Process_Coil_Test();

      // Обновление датчиков через DMA
     // Sensors_Update();
      VL53L5X_Process();

      // Обработка данных IMU
      if (imu_packet_ready)
      {
          imu_packet_ready = 0;
          static uint32_t imu_counter = 0;
          if (++imu_counter >= 100)
          {
              imu_counter = 0;
              Debug_Print(LOG_LEVEL_INFO, "IMU seq=%u r=%d p=%d y=%d\n",
                          last_imu_data.sequence,
                          last_imu_data.roll,
                          last_imu_data.pitch,
                          last_imu_data.yaw);
          }
      }

      // HID Joystick 100Hz Transmitter
      static uint32_t last_hid_time = 0;
      if (HAL_GetTick() - last_hid_time >= 10) {
          last_hid_time = HAL_GetTick();
          Calculate_3D_Position();
          Send_HID_Report();
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

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */

// Обработчик завершения приёма UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
    	if (training_mode) {
    	            // В режиме обучения не обрабатываем команды, просто возобновляем приём
    	            HAL_UART_Receive_IT(&huart2, &RxChar, 1);
    	            return;
    	        }
        // Режим приёма калибровки катушек (flash_coil)
        if (binary_rx_mode) {
            if (binary_rx_index < binary_rx_total) {
                binary_rx_buffer[binary_rx_index++] = RxChar;
            }
            if (binary_rx_index >= binary_rx_total) {
                binary_rx_mode = 0;
                Process_Coil_Calib_Data(binary_rx_buffer, binary_rx_total);
            }
        }
        // Режим приёма токов катушек после команды b_all
        else if (coil_rx_mode) {
            if (coil_rx_index < coil_rx_total) {
                coil_rx_buffer[coil_rx_index++] = RxChar;
                coil_rx_last_byte = HAL_GetTick();  // запомнили время
            }
            if (coil_rx_index >= coil_rx_total) {
                coil_rx_mode = 0;
                float *coil_powers = (float*)coil_rx_buffer;
                // Apply only NUM_COILS values; extra Python-side values are ignored
                for (int i = 0; i < NUM_COILS; i++) {
                    Set_Coil_Power(i, coil_powers[i]);
                }
            }
        }
        // Обычный командный режим (накопление команды)
        else {
            // Обработка символов командной строки
            if (RxChar == '\r' || RxChar == '\n') {
                if (command_index > 0) {
                    new_command = 1;
                }
                if (echo_enabled) {
                    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);
                }
            } else if (RxChar == '\b' || RxChar == 127) {
                if (command_index > 0) {
                    command_index--;
                    if (echo_enabled) {
                        HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, 10);
                    }
                }
            } else if (command_index < sizeof(command_buffer) - 1) {
                command_buffer[command_index++] = RxChar;
                if (echo_enabled) {
                    HAL_UART_Transmit(&huart2, &RxChar, 1, 10);
                }
            }
        }

        // Возобновляем приём следующего байта
        HAL_UART_Receive_IT(&huart2, &RxChar, 1);
    }
}

// Обработчик ошибок UART
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &RxChar, 1);
    }
}

// Обработчик периодического таймера
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        Sensors_Tick();      // обновление датчиков
        control_tick = 1;    // флаг для левитации
    }
}

// Обработчик команд консоли (полная версия)
void Process_Console_Commands(void) {
    if (!new_command) return;
    new_command = 0;

    command_buffer[command_index] = '\0';
    total_commands++;

    char *cmd_start = (char*)command_buffer;
    while (*cmd_start == ' ' || *cmd_start == '\t' || *cmd_start == '\r' || *cmd_start == '\n')
        cmd_start++;

    int len = strlen(cmd_start);
    while (len > 0 && (cmd_start[len-1] == ' ' || cmd_start[len-1] == '\t' ||
                        cmd_start[len-1] == '\r' || cmd_start[len-1] == '\n')) {
        cmd_start[--len] = '\0';
    }

    if (len == 0) {
        command_index = 0;
        memset(command_buffer, 0, sizeof(command_buffer));
        Show_Prompt();
        return;
    }

    char *cmd = strtok(cmd_start, " ");
    if (cmd == NULL) {
        command_index = 0;
        memset(command_buffer, 0, sizeof(command_buffer));
        Show_Prompt();
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        Show_Help_Menu();
    }
    else if (strcmp(cmd, "status") == 0) {
        Show_System_Status();
    }
    else if (strcmp(cmd, "coil") == 0) {
        char *idx_str = strtok(NULL, " ");
        char *power_str = strtok(NULL, " ");
        if (idx_str && power_str) {
            int idx = atoi(idx_str);
            float power = atof(power_str);
            if (idx >= 0 && idx < NUM_COILS) {
                Set_Coil_Power(idx, power);
                Debug_Print(LOG_LEVEL_INFO, "Coil %d set to %.2f\r\n", idx, power);
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Invalid coil index\r\n");
            }
        } else {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: coil <idx> <power>\r\n");
        }
    }
    else if (strcmp(cmd, "coil_all") == 0) {
        char *power_str = strtok(NULL, " ");
        if (power_str) {
            float power = atof(power_str);
            Set_All_Coils_Power(power);
            Debug_Print(LOG_LEVEL_INFO, "All coils set to %.2f\r\n", power);
        } else {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: coil_all <power>\r\n");
        }
    }
    else if (strcmp(cmd, "coil_off") == 0) {
        Stop_All_Coils();
        Debug_Print(LOG_LEVEL_INFO, "All coils turned off\r\n");
    }
    else if (strcmp(cmd, "sensor") == 0) {
        char *arg = strtok(NULL, " ");
        if (!arg) {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: sensor <idx>\r\n");
        } else {
            int idx = atoi(arg);
            if (idx < 0 || idx >= NUM_SENSORS) {
                Debug_Print(LOG_LEVEL_ERROR, "Invalid sensor index\r\n");
            } else {
                Read_Sensor_Calibrated(idx);
            }
        }
    }
    else if (strcmp(cmd, CMD_TEST_COIL) == 0) {
        char *idx_str = strtok(NULL, " ");
        char *start_str = strtok(NULL, " ");
        char *end_str = strtok(NULL, " ");
        char *step_str = strtok(NULL, " ");
        char *dur_str = strtok(NULL, " ");
        if (idx_str && start_str && end_str && step_str && dur_str) {
            int idx = atoi(idx_str);
            float start = atof(start_str);
            float end = atof(end_str);
            float step = atof(step_str);
            uint32_t dur = atoi(dur_str);
            if (idx >= 0 && idx < NUM_COILS) {
                for (int i = 0; i < NUM_COILS; i++) {
                    if (i != idx) Set_Coil_Power(i, 0.0f);
                }
                Start_Coil_Test(idx, start, end, step, dur);
                Debug_Print(LOG_LEVEL_INFO, "Started test on coil %d\r\n", idx);
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Invalid coil index\r\n");
            }
        } else {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: test_coil <idx> <start> <end> <step> <duration_ms>\r\n");
        }
    }
    else if (strcmp(cmd, CMD_TEST_ALL) == 0) {
        char *power_str = strtok(NULL, " ");
        if (power_str) {
            float power = atof(power_str);
            Set_All_Coils_Power(power);
            Debug_Print(LOG_LEVEL_INFO, "Set all coils to %.2f\r\n", power);
        } else {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: test_all <power>\r\n");
        }
    }
    else if (strcmp(cmd, CMD_TEST_GROUP) == 0) {
        char *mask_str = strtok(NULL, " ");
        char *power_str = strtok(NULL, " ");
        if (mask_str && power_str) {
            uint32_t mask = strtoul(mask_str, NULL, 0);
            float power = atof(power_str);
            for (int i = 0; i < NUM_COILS; i++) {
                if (mask & (1 << i)) {
                    Set_Coil_Power(i, power);
                } else {
                    Set_Coil_Power(i, 0.0f);
                }
            }
            Debug_Print(LOG_LEVEL_INFO, "Set mask 0x%X to %.2f\r\n", mask, power);
        } else {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: test_group <mask> <power>\r\n");
        }
    }
    else if (strcmp(cmd, CMD_STOP_TEST) == 0) {
        Stop_Coil_Test();
        Debug_Print(LOG_LEVEL_INFO, "Coil test stopped\r\n");
    }
    else if (strcmp(cmd, "read_raw") == 0) {
        char *arg = strtok(NULL, " ");
        if (!arg) {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: read_raw <idx>\r\n");
        } else {
            int idx = atoi(arg);
            if (idx < 0 || idx >= NUM_SENSORS) {
                Debug_Print(LOG_LEVEL_ERROR, "Invalid sensor index\r\n");
            } else {
                Read_Sensor_With_Gain(idx);
            }
        }
    }
    else if (strcmp(cmd, CMD_COILS_OFF) == 0) {
        Stop_All_Coils();
        Debug_Print(LOG_LEVEL_INFO, "All coils turned off\r\n");
    }
    else if (strcmp(cmd, "read_cal") == 0) {
        char *arg = strtok(NULL, " ");
        if (!arg) {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: read_cal <idx>\r\n");
        } else {
            int idx = atoi(arg);
            if (idx < 0 || idx >= NUM_SENSORS) {
                Debug_Print(LOG_LEVEL_ERROR, "Invalid sensor index\r\n");
            } else {
                Read_Sensor_Calibrated(idx);
            }
        }
    }
    else if (strcmp(cmd, "set_pid_pos") == 0) {
        char *axis_str = strtok(NULL, " ");
        char *kp_str = strtok(NULL, " ");
        char *ki_str = strtok(NULL, " ");
        char *kd_str = strtok(NULL, " ");
        if (axis_str && kp_str && ki_str && kd_str) {
            int axis = atoi(axis_str);
            float kp = atof(kp_str);
            float ki = atof(ki_str);
            float kd = atof(kd_str);
            if (axis >=0 && axis <3) {
                pid_controller.Kp_pos[axis] = kp;
                pid_controller.Ki_pos[axis] = ki;
                pid_controller.Kd_pos[axis] = kd;
                Debug_Print(LOG_LEVEL_INFO, "PID pos[%d] set to %f %f %f\n", axis, kp, ki, kd);
            }
        }
    }
    else if (strcmp(cmd, "set_pid_ori") == 0) {
        // TODO: добавить аналогично
    }
    else if (strcmp(cmd, "calibrate") == 0) {
        char *arg = strtok(NULL, " ");
        if (!arg) {
            Debug_Print(LOG_LEVEL_INFO, "Calibrating all active sensors...\r\n");
            for (int i = 0; i < ACTIVE_SENSORS; i++) {
                if (sensors[i].is_connected) {
                    Calibrate_Offset_Procedure(i);
                }
            }
        } else {
            int idx = atoi(arg);
            if (idx < 0 || idx >= NUM_SENSORS) {
                Debug_Print(LOG_LEVEL_ERROR, "Invalid sensor index\r\n");
            } else {
                Calibrate_Offset_Procedure(idx);
            }
        }
    }
    else if (strcmp(cmd, "save_cal") == 0) {
        Save_Calibration_To_Flash();
    }
    else if (strcmp(cmd, "load_cal") == 0) {
        Load_Calibration_From_Flash();
    }
    else if (strcmp(cmd, "start_stream") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg) {
            int interval = atoi(arg);
            if (interval >= 10 && interval <= 1000) {
                stream_interval_ms = interval;
            }
        }
        streaming_active = 1;
        Debug_Print(LOG_LEVEL_INFO, "Streaming started (interval: %lu ms)\r\n", stream_interval_ms);
    }
    else if (strcmp(cmd, "stop_stream") == 0) {
        streaming_active = 0;
        Debug_Print(LOG_LEVEL_INFO, "Streaming stopped\r\n");
    }
    else if (strcmp(cmd, "levitate") == 0) {
        Start_Levitation();
    }
    else if (strcmp(cmd, "stop_levitate") == 0) {
        Stop_Levitation();
    }
    else if (strcmp(cmd, "flash_coil") == 0) {
        Debug_Print(LOG_LEVEL_INFO, "Ready to receive %d bytes...\r\n", sizeof(CoilCalibData_t));
        binary_rx_mode = 1;
        binary_rx_index = 0;
        binary_rx_total = sizeof(CoilCalibData_t);
        HAL_UART_Transmit(&huart2, (uint8_t*)"G", 1, 100);
    }
    else if (strcmp(cmd, "set_target") == 0) {
        char *x_str = strtok(NULL, " ");
        char *y_str = strtok(NULL, " ");
        char *z_str = strtok(NULL, " ");
        if (x_str && y_str && z_str) {
            float x = atof(x_str);
            float y = atof(y_str);
            float z = atof(z_str);
            Set_Levitation_Target(x, y, z);
        } else {
            Debug_Print(LOG_LEVEL_ERROR, "Usage: set_target x y z\r\n");
        }
    }
    else if (strcmp(cmd, "b_all") == 0) {
        echo_enabled = 0;
        while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {}

        uint32_t sync_marker = 0xDEADBEEF;
        HAL_UART_Transmit(&huart2, (uint8_t*)&sync_marker, 4, 100);

        float sensor_buffer[15];
            for (int i = 0; i < 5; i++) {
                sensor_buffer[i*3]   = sensors[i].magnetic_field[0];
                sensor_buffer[i*3+1] = sensors[i].magnetic_field[1];
                sensor_buffer[i*3+2] = sensors[i].magnetic_field[2];
            }
        HAL_UART_Transmit(&huart2, (uint8_t*)sensor_buffer, sizeof(sensor_buffer), 100);

        HAL_NVIC_DisableIRQ(USART2_IRQn);
        float coil_buffer[12];
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, (uint8_t*)coil_buffer, sizeof(coil_buffer), 500);
        HAL_NVIC_EnableIRQ(USART2_IRQn);

        if (status == HAL_OK) {
            // Apply only NUM_COILS values; protocol may carry 12 for Python compatibility
            for (int i = 0; i < NUM_COILS; i++) {
                Set_Coil_Power(i, coil_buffer[i]);
            }
            // Отправляем синхрометку перед эхом (little-endian: 0xCAFEBABE -> BE BA FE CA)
            while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {}  // ждём завершения предыдущей отправ
            uint32_t echo_marker = 0xCAFEBABE;
            HAL_UART_Transmit(&huart2, (uint8_t*)&echo_marker, 4, 100);
            // Отправляем сами токи (48 байт)
            HAL_UART_Transmit(&huart2, (uint8_t*)coil_buffer, sizeof(coil_buffer), 100);
        }

        echo_enabled = 1;
        command_index = 0;
        memset(command_buffer, 0, sizeof(command_buffer));
        Show_Prompt();
        return;
    }
    else if (strcmp(cmd, "train_on") == 0) {
            training_mode = 1;
            echo_enabled = 0;
            HAL_UART_AbortReceive(&huart2); // Остановить прерывания для чистого бинарного обмена
            Stop_All_Coils();
        }
        else if (strcmp(cmd, "train_off") == 0) {
            training_mode = 0;
            echo_enabled = 1;
            Stop_All_Coils();
            HAL_UART_Receive_IT(&huart2, &RxChar, 1); // Вернуть прерывания для консоли
        }
        else if (strcmp(cmd, "usb_mode") == 0) {
            char *arg = strtok(NULL, " ");
            if (arg) {
                int mode = atoi(arg);
                Switch_USB_Mode(mode);
                Debug_Print(LOG_LEVEL_INFO, "USB Mode switched to %s\r\n", mode == 1 ? "SpaceMouse" : "Joystick");
            } else {
                Debug_Print(LOG_LEVEL_ERROR, "Usage: usb_mode <0=Joystick, 1=SpaceMouse>\r\n");
            }
        }


    else {
           Debug_Print(LOG_LEVEL_ERROR, "Unknown command: '%s'\r\n", cmd);
           Debug_Print(LOG_LEVEL_INFO, "Try 'help' for available commands\r\n");
       }
    command_index = 0;
    memset(command_buffer, 0, sizeof(command_buffer));
    Show_Prompt();
}
/* USER CODE END 4 */

/* MPU Configuration */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

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
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    // Safety first: turn off all coils before halting
    Stop_All_Coils();
    Debug_Print(LOG_LEVEL_ERROR, "Fatal error occurred! System halted.\r\n");
    while(1) {
        HAL_Delay(500);
    }
}

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
