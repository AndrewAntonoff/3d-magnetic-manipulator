#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "main.h"
#include <stdarg.h>

// Уровни логирования (оставьте как есть)
typedef enum {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_VERBOSE
} LogLevel_t;

// Форматы вывода (оставьте как есть)
typedef enum {
    FORMAT_HUMAN = 0,
    FORMAT_CSV,
    FORMAT_JSON,
    FORMAT_BINARY
} OutputFormat_t;

// ---------- КЛЮЧЕВАЯ ЧАСТЬ: условное отключение ----------
#ifndef DEBUG_ENABLED
    #define DEBUG_ENABLED 1   // по умолчанию отладка выключена
#endif

#if DEBUG_ENABLED
    // Режим отладки включён – объявляем реальные функции
    void Debug_Init(UART_HandleTypeDef* huart);
    void Debug_Print(LogLevel_t level, const char* format, ...);
    void Debug_Set_Log_Level(LogLevel_t level);
    void Debug_Set_Output_Format(OutputFormat_t format);
    // ... остальные прототипы
#else
    // Режим отладки выключен – все функции пустые (статические inline, чтобы избежать множественных определений)
    static inline void Debug_Init(UART_HandleTypeDef* huart) { (void)huart; }
    static inline void Debug_Print(LogLevel_t level, const char* format, ...) { (void)level; (void)format; }
    static inline void Debug_Set_Log_Level(LogLevel_t level) { (void)level; }
    static inline void Debug_Set_Output_Format(OutputFormat_t format) { (void)format; }
    // ... аналогично для остальных функций
#endif

// Остальные объявления (если есть) оставить как есть
// ...

#endif /* DEBUG_CONSOLE_H */
