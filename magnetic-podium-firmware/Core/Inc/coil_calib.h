#ifndef COIL_CALIB_H
#define COIL_CALIB_H

#include <stdint.h>

#define COIL_CALIB_SIGNATURE 0xCAFEBABE
#define COIL_CALIB_VERSION   1
#define COIL_CALIB_NUM_COILS   12
#define COIL_CALIB_NUM_SENSORS 5
#define COIL_CALIB_NUM_POINTS  13
#define COIL_CALIB_FLASH_ADDR  0x00010000

typedef struct {
    uint32_t signature;
    uint32_t version;
    uint32_t num_coils;
    uint32_t num_sensors;
    uint32_t num_points;
} CoilCalibHeader_t;

typedef struct {
    CoilCalibHeader_t header;
    float currents[COIL_CALIB_NUM_POINTS];
    float field[COIL_CALIB_NUM_COILS][COIL_CALIB_NUM_POINTS][COIL_CALIB_NUM_SENSORS][3];
} CoilCalibData_t;

void Load_Coil_Calibration(void);
void Get_Coil_Field(uint8_t coil, float I, float field[5][3]);

#endif
