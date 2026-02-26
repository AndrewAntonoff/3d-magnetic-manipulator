#include "coil_calib.h"
#include "qspi_flash.h"
#include "debug_console.h"

static CoilCalibData_t calib_data;
static uint8_t calib_loaded ;

void Load_Coil_Calibration(void) {
    CoilCalibHeader_t header;
    QSPI_Flash_ReadBuffer(COIL_CALIB_FLASH_ADDR, (uint8_t*)&header, sizeof(header));

    if (header.signature != COIL_CALIB_SIGNATURE || header.version != COIL_CALIB_VERSION) {
        Debug_Print(LOG_LEVEL_ERROR, "Coil calibration signature/version mismatch!\r\n");
        return;
    }
    if (header.num_coils != COIL_CALIB_NUM_COILS ||
        header.num_sensors != COIL_CALIB_NUM_SENSORS ||
        header.num_points != COIL_CALIB_NUM_POINTS) {
        Debug_Print(LOG_LEVEL_ERROR, "Coil calibration size mismatch!\r\n");
        return;
    }

    QSPI_Flash_ReadBuffer(COIL_CALIB_FLASH_ADDR, (uint8_t*)&calib_data, sizeof(CoilCalibData_t));
    calib_loaded = 1;
    Debug_Print(LOG_LEVEL_INFO, "Coil calibration loaded (%d coils, %d points)\r\n",
                calib_data.header.num_coils, calib_data.header.num_points);
}

void Get_Coil_Field(uint8_t coil, float I, float field[5][3]) {
    if (!calib_loaded) {
        for (int s = 0; s < 5; s++)
            for (int k = 0; k < 3; k++)
                field[s][k] = 0.0f;
        return;
    }

    float I_min = calib_data.currents[0];
    float I_max = calib_data.currents[COIL_CALIB_NUM_POINTS-1];
    if (I < I_min) I = I_min;
    if (I > I_max) I = I_max;

    int idx = 0;
    while (idx < COIL_CALIB_NUM_POINTS-1 && I > calib_data.currents[idx+1]) {
        idx++;
    }

    if (I <= calib_data.currents[0]) {
        for (int s = 0; s < 5; s++)
            for (int k = 0; k < 3; k++)
                field[s][k] = calib_data.field[coil][0][s][k];
    } else if (I >= calib_data.currents[COIL_CALIB_NUM_POINTS-1]) {
        for (int s = 0; s < 5; s++)
            for (int k = 0; k < 3; k++)
                field[s][k] = calib_data.field[coil][COIL_CALIB_NUM_POINTS-1][s][k];
    } else {
        float t = (I - calib_data.currents[idx]) / (calib_data.currents[idx+1] - calib_data.currents[idx]);
        for (int s = 0; s < 5; s++) {
            for (int k = 0; k < 3; k++) {
                float v0 = calib_data.field[coil][idx][s][k];
                float v1 = calib_data.field[coil][idx+1][s][k];
                field[s][k] = v0 + t * (v1 - v0);
            }
        }
    }
}
