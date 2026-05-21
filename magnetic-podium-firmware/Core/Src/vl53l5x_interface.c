#include "vl53l5x_interface.h"
#include "debug_console.h"

// Этот файл является оберткой (wrapper) для библиотеки ST ULD VL53L5X.
// Для того, чтобы код компилировался и работал:
// 1. Сгенерируйте и добавьте файлы драйвера VL53L5X из ST (vl53l5cx_api.c/.h и т.д.)
// 2. Убедитесь, что массив прошивки (vl53l5cx_firmware) помещен в секцию .qspi_data.
// В platform.c драйвера ST ULD укажите: 
// #pragma GCC attribute push("ExtQSPIFlashSection")
// const uint8_t vl53l5cx_firmware[] = { ... };
// #pragma GCC attribute pop

// Пример включения заголовков ULD ST
#include "vl53l5cx_api.h"
/* Note: i2c.h is already included via platform.h → vl53l5cx_api.h */
static VL53L5CX_Configuration Dev;
static uint8_t is_ranging = 0;

static float last_distance_mm = 0.0f;

uint8_t VL53L5X_Init(void) {
    Debug_Print(LOG_LEVEL_INFO, "Initializing VL53L5X...\r\n");

    uint8_t status = 0;
    uint8_t isAlive = 0;

    /* VL53L5CX_Platform in this project has only one field: address.
       The I2C handle (hi2c1) is used globally by the platform functions
       in VL53L5X/src/platform.c — no explicit assignment needed here. */
    Dev.platform.address = 0x52;  // VL53L5CX default I2C address (7-bit)

    status = vl53l5cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status != 0) {
        Debug_Print(LOG_LEVEL_ERROR, "VL53L5X not detected\r\n");
        return 0;
    }
    
    status = vl53l5cx_init(&Dev); // Здесь загружается прошивка из QSPI Flash
    if(status) {
        Debug_Print(LOG_LEVEL_ERROR, "VL53L5X Init Failed!\r\n");
        return 0;
    }
    
    // Установка разрешения (4x4 или 8x8) и частоты
    vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
    vl53l5cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l5cx_start_ranging(&Dev);
    is_ranging = 1;
    Debug_Print(LOG_LEVEL_INFO, "VL53L5X Started Ranging\r\n");
    
    return 1;
}

void VL53L5X_Process(void) {
    // В этом методе периодически считываем данные с датчика
    
    if (!is_ranging) return;
    
    uint8_t isReady = 0;
    vl53l5cx_check_data_ready(&Dev, &isReady);
    if(isReady) {
        VL53L5CX_ResultsData Results;
        vl53l5cx_get_ranging_data(&Dev, &Results);
        
        // Для магнитной платформы нас интересует высота (в центре чаши).
        // Можно усреднить данные по зонам или взять самую уверенную зону по центру:
        // Зоны 5, 6, 9, 10 для матрицы 4х4 — это центральные области.
        long sum_distance = 0;
        int valid_zones = 0;
        
        for (int i = 0; i < 16; i++) {
            if (Results.target_status[i] == 5 || Results.target_status[i] == 9) { // 5=good, 9=ok
                sum_distance += Results.distance_mm[i];
                valid_zones++;
            }
        }
        
        if (valid_zones > 0) {
            last_distance_mm = (float)sum_distance / valid_zones;
        }
    }
}

float VL53L5X_GetDistance(void) {
    if (last_distance_mm <= 0.1f) {
        last_distance_mm = 55.0f; // Возврат безопасного "дома" пока нет данных
    }
    return last_distance_mm;
}
