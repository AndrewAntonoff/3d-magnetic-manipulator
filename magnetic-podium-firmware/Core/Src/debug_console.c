#include "debug_console.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef* debug_uart = NULL;
static LogLevel_t current_log_level = LOG_LEVEL_INFO;
static OutputFormat_t current_format = FORMAT_HUMAN;

void Debug_Init(UART_HandleTypeDef* huart) {
    debug_uart = huart;
}

void Debug_Print(LogLevel_t level, const char* format, ...) {
    if(level > current_log_level || debug_uart == NULL) {
        return;
    }

    char buffer[256];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Убираем лишние пробелы и перенос строки в начале
    // Добавляем timestamp
    char timed_buffer[300];
    snprintf(timed_buffer, sizeof(timed_buffer), "[%lu] %s\r\n",
             (unsigned long)HAL_GetTick(), buffer);

    HAL_UART_Transmit(debug_uart, (uint8_t*)timed_buffer, strlen(timed_buffer), 100);
}

void Debug_Set_Log_Level(LogLevel_t level) {
    current_log_level = level;
}

void Debug_Set_Output_Format(OutputFormat_t format) {
    current_format = format;
}

void Console_Process_Command(const char* command) {
    // Реализация обработки команд (используется в main.c)
}

void Console_Show_Help(void) {
    // Реализация показа справки (используется в main.c)
}

void Console_Show_Status(void) {
    // Реализация показа статуса (используется в main.c)
}

void Start_RealTime_Monitoring(uint32_t interval_ms) {
    // Реализация начала мониторинга
}

void Stop_RealTime_Monitoring(void) {
    // Реализация остановки мониторинга
}

void Set_Monitoring_Interval(uint32_t interval_ms) {
    // Реализация установки интервала
}

void Export_Coil_Data_CSV(void) {
    Debug_Print(LOG_LEVEL_INFO, "Exporting coil data to CSV...\r\n");
    // Реализация экспорта
}

void Export_Sensor_Data_CSV(void) {
    Debug_Print(LOG_LEVEL_INFO, "Exporting sensor data to CSV...\r\n");
    // Реализация экспорта
}

void Export_System_Log(void) {
    Debug_Print(LOG_LEVEL_INFO, "Exporting system log...\r\n");
    // Реализация экспорта лога
}
