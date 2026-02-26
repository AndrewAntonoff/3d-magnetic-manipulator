#include "debug_console.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#if DEBUG_ENABLED

static UART_HandleTypeDef* debug_uart = NULL;
static LogLevel_t current_log_level = LOG_LEVEL_INFO;
static OutputFormat_t current_format = FORMAT_HUMAN;

void Debug_Init(UART_HandleTypeDef* huart) {
    debug_uart = huart;
}

void Debug_Print(LogLevel_t level, const char* format, ...) {
    if(level > current_log_level || debug_uart == NULL) return;
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    char timed_buffer[300];
    snprintf(timed_buffer, sizeof(timed_buffer), "[%lu] %s\r\n",
             (unsigned long)HAL_GetTick(), buffer);
    HAL_UART_Transmit(debug_uart, (uint8_t*)timed_buffer, strlen(timed_buffer), 100);
}

// ... остальные реализации Debug_Set_Log_Level и т.д.

#else
    // Пустой файл, если DEBUG_ENABLED = 0 – можно вообще ничего не писать,
    // но чтобы линкер не ругался на отсутствие объектного файла,
    // оставим заглушку или просто удалим этот файл из сборки при выключенной отладке.
    // Лучше всё же оставить пустой файл, чтобы не менять структуру проекта.
    // Можно добавить фиктивную функцию, которая никогда не вызывается, но это необязательно.
#endif
