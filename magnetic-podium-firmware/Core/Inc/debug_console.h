#ifndef __DEBUG_CONSOLE_H
#define __DEBUG_CONSOLE_H

#include "main.h"
#include <stdarg.h>

// Уровни логирования
typedef enum {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_VERBOSE
} LogLevel_t;

// Форматы вывода
typedef enum {
    FORMAT_HUMAN = 0,
    FORMAT_CSV,
    FORMAT_JSON,
    FORMAT_BINARY
} OutputFormat_t;

// Прототипы функций
void Debug_Init(UART_HandleTypeDef* huart);
void Debug_Print(LogLevel_t level, const char* format, ...);
void Debug_Print_Raw(const char* format, ...);
void Debug_Set_Log_Level(LogLevel_t level);
void Debug_Set_Output_Format(OutputFormat_t format);

// Командная консоль
void Console_Process_Command(const char* command);
void Console_Show_Help(void);
void Console_Show_Status(void);

// Мониторинг в реальном времени
void Start_RealTime_Monitoring(uint32_t interval_ms);
void Stop_RealTime_Monitoring(void);
void Set_Monitoring_Interval(uint32_t interval_ms);

// Экспорт данных
void Export_Coil_Data_CSV(void);
void Export_Sensor_Data_CSV(void);
void Export_System_Log(void);

#endif /* __DEBUG_CONSOLE_H */
