#include "levitation_control.h"
#include "config.h"
#include "main.h"          // для доступа к last_imu_data, imu_packet_ready
#include "coil_driver.h"
#include "sensor_mlx90393.h"
#include "debug_console.h"
#include "coil_calib.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

extern MLX90393_t sensors[NUM_SENSORS];

// Поле постоянного магнита (измерено без шара)
static const float B_permanent[5][3] = {
    {13076.9f, 2180.7f, 6859.3f},   // датчик 0 (уже)
    {10834.9f, 3748.3f, -15988.2f}, // датчик 1
    {17922.4f, -208.4f, -24747.2f}, // датчик 2
    {-10471.3f, -2354.4f, 21186.9f},// датчик 3
    {-6204.0f, 313.3f, 18233.3f}    // датчик 4

};

// ------------------------------------------------------------------
// Константы
// ------------------------------------------------------------------
#define MU0_4PI 1e-7f                     // μ0/(4π) в СИ
#define HOME_Z          25.0f              // домашняя высота (мм)
#define RELEASE_DIST_THRESH  2.0f          // порог расстояния для отпускания (мм)
#define RELEASE_SPEED_THRESH 5.0f          // порог скорости (мм/с)
#define K_FORCE 2.0f
#define BALL_WEIGHT_N 1.7f   // вес шара в ньютонах (подберите по массе)

// ------------------------------------------------------------------
// Глобальные переменные
// ------------------------------------------------------------------
SystemState_t system_state = {
    .coils_enabled = 0,
    .sensors_enabled = 0,
    .monitoring_active = 0,
    .calibration_done = 0,
    .levitation_active = 0,
    .system_uptime_ms = 0,
    .cpu_usage_percent = 0.0f,
    .ball_position = {0.0f, 0.0f, 0.0f}
};

OperationMode_t current_mode = MODE_IDLE;

// Геометрия катушек (в мм)
static CoilGeometry_t coil_geometry[NUM_COILS];

float PowerToCurrent(float power);

// ------------------------------------------------------------------
// Вспомогательные функции (математика поля)
// ------------------------------------------------------------------

static void Compute_ForceTorque(uint8_t coil_idx, const float ball_pos[3],
                                const float ball_moment[3],
                                float force[3], float torque[3])
{
    float r_c[3] = {coil_geometry[coil_idx].x,
                    coil_geometry[coil_idx].y,
                    coil_geometry[coil_idx].z};
    float r[3] = {
        (ball_pos[0] - r_c[0]) / 1000.0f,
        (ball_pos[1] - r_c[1]) / 1000.0f,
        (ball_pos[2] - r_c[2]) / 1000.0f
    };
    float r2 = r[0]*r[0] + r[1]*r[1] + r[2]*r[2];
    float r_norm = sqrtf(r2);
    if (r_norm < 1e-6f) {
        force[0] = force[1] = force[2] = 0;
        torque[0] = torque[1] = torque[2] = 0;
        return;
    }

    float r5 = r2 * r2 * r_norm;

    float Mr = ball_moment[0]*r[0] + ball_moment[1]*r[1] + ball_moment[2]*r[2];
    float mr = coil_geometry[coil_idx].orientation[0]*r[0] +
               coil_geometry[coil_idx].orientation[1]*r[1] +
               coil_geometry[coil_idx].orientation[2]*r[2];
    float Mm = ball_moment[0]*coil_geometry[coil_idx].orientation[0] +
               ball_moment[1]*coil_geometry[coil_idx].orientation[1] +
               ball_moment[2]*coil_geometry[coil_idx].orientation[2];

    float factor = 3.0f / r5;
    for (int i = 0; i < 3; i++) {
        force[i] = factor * ( Mr * coil_geometry[coil_idx].orientation[i] +
                              mr * ball_moment[i] +
                              Mm * r[i] -
                              5.0f * Mr * mr * r[i] / r2 );
    }

    float B[3];
    float r3 = r2 * r_norm;
    factor = 1.0f / r3;
    float coeff = 3.0f * mr / r2;
    for (int i = 0; i < 3; i++) {
        B[i] = factor * (coeff * r[i] - coil_geometry[coil_idx].orientation[i]);
    }

    torque[0] = ball_moment[1]*B[2] - ball_moment[2]*B[1];
    torque[1] = ball_moment[2]*B[0] - ball_moment[0]*B[2];
    torque[2] = ball_moment[0]*B[1] - ball_moment[1]*B[0];

    for (int i = 0; i < 3; i++) {
        force[i]  *= MU0_4PI;
        torque[i] *= MU0_4PI;
    }
}

static void Build_Matrix_A(const float ball_pos[3], const float ball_moment[3],
                           float A[6][12])
{
    for (int j = 0; j < NUM_COILS; j++) {
        float f[3], t[3];
        Compute_ForceTorque(j, ball_pos, ball_moment, f, t);
        for (int i = 0; i < 3; i++) {
            A[i][j]   = f[i];
            A[i+3][j] = t[i];
        }
    }
}

static int InvertMatrix_6x6(const float A[6][6], float inv[6][6])
{
    double aug[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            aug[i][j] = (double)A[i][j];
            aug[i][j+6] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int col = 0; col < 6; col++) {
        int pivot = col;
        double max_val = fabs(aug[col][col]);
        for (int row = col+1; row < 6; row++) {
            if (fabs(aug[row][col]) > max_val) {
                max_val = fabs(aug[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-12) return 0;

        if (pivot != col) {
            for (int j = 0; j < 12; j++) {
                double tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }

        double div = aug[col][col];
        for (int j = 0; j < 12; j++) {
            aug[col][j] /= div;
        }

        for (int row = 0; row < 6; row++) {
            if (row != col) {
                double factor = aug[row][col];
                for (int j = 0; j < 12; j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            inv[i][j] = (float)aug[i][j+6];
        }
    }
    return 1;
}

static void Solve_Currents(const float A[6][12], const float u[6],
                           float I[12], float lambda)
{
    lambda = 0.001f;
    float C[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            float sum = 0;
            for (int k = 0; k < NUM_COILS; k++) {
                sum += A[i][k] * A[j][k];
            }
            C[i][j] = sum;
        }
    }

    for (int i = 0; i < 6; i++) {
        C[i][i] += lambda * lambda;
    }

    float C_inv[6][6];
    if (!InvertMatrix_6x6(C, C_inv)) {
        Debug_Print(LOG_LEVEL_ERROR, "Matrix inversion failed!\n");
        memset(I, 0, NUM_COILS * sizeof(float));
        return;
    }

    float tmp[6];
    for (int i = 0; i < 6; i++) {
        tmp[i] = 0;
        for (int j = 0; j < 6; j++) {
            tmp[i] += C_inv[i][j] * u[j];
        }
    }

    for (int k = 0; k < NUM_COILS; k++) {
        I[k] = 0;
        for (int i = 0; i < 6; i++) {
            I[k] += A[i][k] * tmp[i];
        }
    }

    float max_abs = 0.0f;
    for (int k = 0; k < NUM_COILS; k++) {
        float abs_val = fabsf(I[k]);
        if (abs_val > max_abs) max_abs = abs_val;
    }
    if (max_abs > 1.0f) {
        static uint32_t last_warning = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_warning > 1000) {
            last_warning = now;
            Debug_Print(LOG_LEVEL_WARNING, "Current limit reached, scaling by %.3f\n", 1.0f/max_abs);
        }
        float scale = 1.0f / max_abs;
        for (int k = 0; k < NUM_COILS; k++) {
            I[k] *= scale;
        }
    }
}

// ------------------------------------------------------------------
// Основные функции управления
// ------------------------------------------------------------------

void Initialize_Coil_Geometry(void) {
    const float center_z = 55.0f;

    for (int i = 0; i < 4; i++) {
        float theta = i * (M_PI / 2.0f);
        coil_geometry[i].x = 25.0f * cosf(theta);
        coil_geometry[i].y = 25.0f * sinf(theta);
        coil_geometry[i].z = 7.0f;

        float dx = -coil_geometry[i].x;
        float dy = -coil_geometry[i].y;
        float dz = center_z - coil_geometry[i].z;
        float norm = sqrtf(dx*dx + dy*dy + dz*dz);
        coil_geometry[i].orientation[0] = dx / norm;
        coil_geometry[i].orientation[1] = dy / norm;
        coil_geometry[i].orientation[2] = dz / norm;
    }

    for (int i = 0; i < 8; i++) {
        float theta = i * (M_PI / 4.0f);
        coil_geometry[i+4].x = 45.0f * cosf(theta);
        coil_geometry[i+4].y = 45.0f * sinf(theta);
        coil_geometry[i+4].z = 30.0f;

        float dx = -coil_geometry[i+4].x;
        float dy = -coil_geometry[i+4].y;
        float dz = center_z - coil_geometry[i+4].z;
        float norm = sqrtf(dx*dx + dy*dy + dz*dz);
        coil_geometry[i+4].orientation[0] = dx / norm;
        coil_geometry[i+4].orientation[1] = dy / norm;
        coil_geometry[i+4].orientation[2] = dz / norm;
    }

    Debug_Print(LOG_LEVEL_INFO, "Coil geometry OK (center_z=55 mm)\r\n");
}

void Initialize_Sensor_Geometry(void) {
    sensors[0].geometry.x = 0.0f;
    sensors[0].geometry.y = 0.0f;
    sensors[0].geometry.z = 2.0f;

    float r = 35.0f;
    float z = 15.0f;
    float angles[4] = {45.0f, 135.0f, 225.0f, 315.0f};

    for (int i = 0; i < 4; i++) {
        float theta = angles[i] * M_PI / 180.0f;
        sensors[i+1].geometry.x = r * cosf(theta);
        sensors[i+1].geometry.y = r * sinf(theta);
        sensors[i+1].geometry.z = z;
    }

    Debug_Print(LOG_LEVEL_INFO, "Sensor geometry OK (r=35 mm @ 45°)\r\n");
}

void EstimateBallPosition(float ball_pos[3], const float orientation[4])
{
    // 1. Скорректированные показания датчиков (вычитаем поле катушек)
    float corrected_field[ACTIVE_SENSORS][3];
    float coil_field_cache[NUM_COILS][5][3];

    float coil_currents[NUM_COILS];
    for (int c = 0; c < NUM_COILS; c++) {
        float power = Get_Coil_SignedPower(c);
        coil_currents[c] = PowerToCurrent(power);
    }

    for (int c = 0; c < NUM_COILS; c++) {
        Get_Coil_Field(c, coil_currents[c], coil_field_cache[c]);
    }

    for (int s = 0; s < ACTIVE_SENSORS; s++) {
        float coil_sum[3] = {0, 0, 0};
        for (int c = 0; c < NUM_COILS; c++) {
            coil_sum[0] += coil_field_cache[c][s][0];
            coil_sum[1] += coil_field_cache[c][s][1];
            coil_sum[2] += coil_field_cache[c][s][2];
        }
        corrected_field[s][0] = sensors[s].magnetic_field[0] - coil_sum[0];
        corrected_field[s][1] = sensors[s].magnetic_field[1] - coil_sum[1];
        corrected_field[s][2] = sensors[s].magnetic_field[2] - coil_sum[2];
    }

    // Вычитаем поле постоянного магнита
    for (int s = 0; s < ACTIVE_SENSORS; s++) {
        corrected_field[s][0] -= B_permanent[s][0];
        corrected_field[s][1] -= B_permanent[s][1];
        corrected_field[s][2] -= B_permanent[s][2];
    }

    // === НОВАЯ ОЦЕНКА ВЫСОТЫ ПО НИЖНЕМУ ДАТЧИКУ (датчик 0) ===
    // Используем модуль вектора, так как он менее чувствителен к ориентации
    float B_mag = sqrtf(corrected_field[0][0]*corrected_field[0][0] +
                        corrected_field[0][1]*corrected_field[0][1] +
                        corrected_field[0][2]*corrected_field[0][2]);
    float z_est;
    if (B_mag < 10.0f) {
        // Слишком слабое поле – аварийно устанавливаем высоту
        z_est = 25.0f;
    } else {
        // Константа Kz откалибрована по измерению на высоте 25 мм:
        // |B| = 20512.5 µT, (25-2)=23 мм, Kz = 20512.5 * 23^3 = 249575587.5
        const float Kz = 249600000.0f; // ≈ 2.496e8
        // Правильная формула: z = (Kz / |B|)^(1/3) + 2.0
        z_est = powf(Kz / B_mag, 1.0f/3.0f) + 2.0f;
    }

    // === ОЦЕНКА XY ПО БОКОВЫМ ДАТЧИКАМ (взвешенное среднее) ===
    float weights[ACTIVE_SENSORS];
    float total_weight = 0.0f;
    for (int i = 1; i < ACTIVE_SENSORS; i++) { // используем только датчики 1-4
        if (sensors[i].is_connected) {
            float B2 = corrected_field[i][0]*corrected_field[i][0] +
                       corrected_field[i][1]*corrected_field[i][1] +
                       corrected_field[i][2]*corrected_field[i][2];
            if (B2 > 1.0f) {
                weights[i] = B2;
                total_weight += weights[i];
            } else {
                weights[i] = 0.0f;
            }
        } else {
            weights[i] = 0.0f;
        }
    }

    float rough_x = 0, rough_y = 0;
    if (total_weight > 0.0f) {
        for (int i = 1; i < ACTIVE_SENSORS; i++) {
            rough_x += sensors[i].geometry.x * weights[i];
            rough_y += sensors[i].geometry.y * weights[i];
        }
        rough_x /= total_weight;
        rough_y /= total_weight;
    } else {
        rough_x = 0.0f;
        rough_y = 0.0f;
    }

    // 3. Фильтрация (экспоненциальное сглаживание)
    static float filtered_x = 0.0f, filtered_y = 0.0f, filtered_z = 10.0f;
    const float alpha_xy = 0.85f;
    const float alpha_z = 0.7f;

    filtered_x = alpha_xy * rough_x + (1.0f - alpha_xy) * filtered_x;
    filtered_y = alpha_xy * rough_y + (1.0f - alpha_xy) * filtered_y;
    filtered_z = alpha_z * z_est + (1.0f - alpha_z) * filtered_z;

    ball_pos[0] = filtered_x;
    ball_pos[1] = filtered_y;
    ball_pos[2] = filtered_z;

    // 4. Аварийное ограничение
    float xy = sqrtf(ball_pos[0]*ball_pos[0] + ball_pos[1]*ball_pos[1]);
    if (xy > 55.0f) {
        float s = 55.0f / xy;
        ball_pos[0] *= s;
        ball_pos[1] *= s;
    }
    if (ball_pos[2] < 5.0f) ball_pos[2] = 5.0f;
    if (ball_pos[2] > 40.0f) ball_pos[2] = 40.0f;

    // Периодическая отладка (раз в 500 вызовов)
    static uint32_t counter = 0;
    if (++counter >= 500) {
        counter = 0;
        Debug_Print(LOG_LEVEL_INFO, "Est: (%.1f, %.1f, %.1f) |B0|=%.0f\n",
                    ball_pos[0], ball_pos[1], ball_pos[2], B_mag);
    }
}

// ------------------------------------------------------------------
// ПИД-регулятор (6 степеней свободы)
// ------------------------------------------------------------------
PID_6DOF_t pid_controller = {
    .Kp_pos = {15.0f, 12.0f, 1.5f},   // Z увеличено
    .Ki_pos = {0.001f, 0.001f, 0.01f},
    .Kd_pos = {8.0f, 8.0f, 0.05f},
    .integral_pos = {0},
    .prev_error_pos = {0},
    .setpoint_pos = {0.0f, 0.0f, 25.0f},
    .output_pos = {0},

    .Kp_ori = {0.05f, 0.05f, 0.02f},
    .Ki_ori = {0.005f, 0.005f, 0.002f},
    .Kd_ori = {0.02f, 0.02f, 0.01f},
    .integral_ori = {0},
    .prev_error_ori = {0},
    .setpoint_ori = {0,0,0},
    .output_ori = {0},

    .max_integral = 15.0f,
    .max_output   = 2.0f
};

void Update_PID_Controller(float dt)
{
    IMU_Data_t imu;
    __disable_irq();
    memcpy(&imu, (void*)&last_imu_data, sizeof(IMU_Data_t));
    __enable_irq();

    float roll  = imu.roll  * 0.001f;
    float pitch = imu.pitch * 0.001f;
    float yaw   = imu.yaw   * 0.001f;

    float gyro_x = imu.gyro_x * 0.001f * (M_PI/180.0f);
    float gyro_y = imu.gyro_y * 0.001f * (M_PI/180.0f);
    float gyro_z = imu.gyro_z * 0.001f * (M_PI/180.0f);

    static uint32_t last_imu_debug = 0;
    if (HAL_GetTick() - last_imu_debug > 5000) {
        last_imu_debug = HAL_GetTick();
        Debug_Print(LOG_LEVEL_INFO, "IMU angles: roll=%.2f pitch=%.2f yaw=%.2f\n",
                    roll, pitch, yaw);
    }

    // --- Ошибка по позиции ---
    float error_pos[3] = {
        pid_controller.setpoint_pos[0] - system_state.ball_position[0],
        pid_controller.setpoint_pos[1] - system_state.ball_position[1],
        pid_controller.setpoint_pos[2] - system_state.ball_position[2]
    };

    for (int i = 0; i < 3; i++) {
        pid_controller.output_pos[i] = pid_controller.Kp_pos[i] * error_pos[i];

        pid_controller.integral_pos[i] += error_pos[i] * dt;
        if (pid_controller.integral_pos[i] >  pid_controller.max_integral) pid_controller.integral_pos[i] =  pid_controller.max_integral;
        if (pid_controller.integral_pos[i] < -pid_controller.max_integral) pid_controller.integral_pos[i] = -pid_controller.max_integral;
        pid_controller.output_pos[i] += pid_controller.Ki_pos[i] * pid_controller.integral_pos[i];

        if (dt > 0.001f) {
            float derivative = (error_pos[i] - pid_controller.prev_error_pos[i]) / dt;
            pid_controller.output_pos[i] += pid_controller.Kd_pos[i] * derivative;
            pid_controller.prev_error_pos[i] = error_pos[i];
        }

        if (pid_controller.output_pos[i] >  pid_controller.max_output) pid_controller.output_pos[i] =  pid_controller.max_output;
        if (pid_controller.output_pos[i] < -pid_controller.max_output) pid_controller.output_pos[i] = -pid_controller.max_output;
    }

    // --- Ошибка по ориентации ---
    float error_ori[3] = {
        pid_controller.setpoint_ori[0] - roll,
        pid_controller.setpoint_ori[1] - pitch,
        pid_controller.setpoint_ori[2] - yaw
    };

    for (int i = 0; i < 3; i++) {
        pid_controller.output_ori[i] = pid_controller.Kp_ori[i] * error_ori[i];

        pid_controller.integral_ori[i] += error_ori[i] * dt;
        if (pid_controller.integral_ori[i] >  pid_controller.max_integral) pid_controller.integral_ori[i] =  pid_controller.max_integral;
        if (pid_controller.integral_ori[i] < -pid_controller.max_integral) pid_controller.integral_ori[i] = -pid_controller.max_integral;
        pid_controller.output_ori[i] += pid_controller.Ki_ori[i] * pid_controller.integral_ori[i];

        float angular_velocity = (i==0) ? gyro_x : (i==1) ? gyro_y : gyro_z;
        pid_controller.output_ori[i] += pid_controller.Kd_ori[i] * angular_velocity;

        if (pid_controller.output_ori[i] >  pid_controller.max_output) pid_controller.output_ori[i] =  pid_controller.max_output;
        if (pid_controller.output_ori[i] < -pid_controller.max_output) pid_controller.output_ori[i] = -pid_controller.max_output;

        pid_controller.prev_error_ori[i] = error_ori[i];
    }
}

void Calculate_Coil_Forces(float fx, float fy, float fz,
                           float tx, float ty, float tz,
                           float* coil_powers)
{
    float ball_pos[3] = {system_state.ball_position[0],
                         system_state.ball_position[1],
                         system_state.ball_position[2]};

    // Магнитный момент шара (калиброванный, подставьте свой)
    float ball_moment[3] = {-15.0f, -6.0f, 8.0f};
    float A[6][12];
    Build_Matrix_A(ball_pos, ball_moment, A);

    float u[6] = {fx, fy, fz, tx, ty, tz};
    // ВРЕМЕННО: отключаем моменты для теста (3DoF)
    for (int i = 3; i < 6; i++) {
        u[i] = 0.0f;
        for (int j = 0; j < NUM_COILS; j++) {
            A[i][j] = 0.0f;
        }
    }
    float I[12];
    Solve_Currents(A, u, I, 0.01f);

    for (int i = 0; i < NUM_COILS; i++) {
        coil_powers[i] = I[i] * K_FORCE;
    }

    static uint32_t counter = 0;
    if (++counter >= 500) {
        counter = 0;
        Debug_Print(LOG_LEVEL_INFO, "Raw currents: I0=%.3f I1=%.3f I2=%.3f\n", I[0], I[1], I[2]);
    }
}

void Apply_Levitation_Control(void)
{
    static uint32_t last_profile = 0;
    uint32_t t_start = HAL_GetTick();

    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (last_time == 0) { last_time = current_time; return; }
    float dt = (current_time - last_time) / 1000.0f;
    if (dt < 1e-6f) return;
    if (dt > 0.1f) dt = 0.1f;

    float x = system_state.ball_position[0];
    float y = system_state.ball_position[1];
    float z = system_state.ball_position[2];
    float xy_dist = sqrtf(x*x + y*y);
    if (xy_dist > 55.0f || z < 0.0f || z > 40.0f) {
        Debug_Print(LOG_LEVEL_WARNING, "Ball out of bounds (x=%.1f, y=%.1f, z=%.1f). Stopping levitation.\r\n",
                    x, y, z);
        Stop_Levitation();
        return;
    }

    Update_PID_Controller(dt);

    // Формирование желаемых сил (вес учтён)
    float fx = pid_controller.output_pos[0] * 12.0f;
    float fy = pid_controller.output_pos[1] * 12.0f;
    float fz = pid_controller.output_pos[2] * 0.3f + BALL_WEIGHT_N;

    if (z < 18.0f) {
        fz += 0.3f;
    }

    float tx = pid_controller.output_ori[0];
    float ty = pid_controller.output_ori[1];
    float tz = pid_controller.output_ori[2];

    // Ограничения (увеличены для горизонтали)
    if (fabsf(fx) > 5.0f) fx = 5.0f * (fx > 0 ? 1.0f : -1.0f);
    if (fabsf(fy) > 5.0f) fy = 5.0f * (fy > 0 ? 1.0f : -1.0f);
    if (fabsf(fz) > 5.0f) fz = 5.0f * (fz > 0 ? 1.0f : -1.0f);

    float coil_powers[NUM_COILS];
    Calculate_Coil_Forces(fx, fy, fz, tx, ty, tz, coil_powers);

    // Мониторинг максимального тока
    static uint32_t current_monitor_counter = 0;
    if (++current_monitor_counter >= 200) {
        current_monitor_counter = 0;
        float max_current_rel = 0.0f;
        for (int i = 0; i < NUM_COILS; i++) {
            float abs_current = fabsf(coil_powers[i]);
            if (abs_current > max_current_rel) max_current_rel = abs_current;
        }
        float max_current_amp = max_current_rel * COIL_MAX_CURRENT;
        Debug_Print(LOG_LEVEL_INFO, "Max current: %.2f A (rel=%.2f)\r\n", max_current_amp, max_current_rel);
    }

    // Фильтр токов
    static float filtered_coil_powers[NUM_COILS] = {0};
    const float alpha = 0.7f;
    for (int i = 0; i < NUM_COILS; i++) {
        filtered_coil_powers[i] = alpha * coil_powers[i] + (1.0f - alpha) * filtered_coil_powers[i];
        coil_powers[i] = filtered_coil_powers[i];
    }

    for (int i = 0; i < NUM_COILS; i++) {
        Set_Coil_Power(i, coil_powers[i]);
    }

    static uint32_t dbg_counter = 0;
    if (++dbg_counter % 100 == 0) {
        Debug_Print(LOG_LEVEL_INFO, "z=%.1f fz=%.2f I0=%.2fA I1=%.2fA I2=%.2fA\n",
                    system_state.ball_position[2], fz,
                    PowerToCurrent(coil_powers[0]),
                    PowerToCurrent(coil_powers[1]),
                    PowerToCurrent(coil_powers[2]));
    }

    last_time = current_time;

    uint32_t t_end = HAL_GetTick();
    if (t_end - last_profile > 1000) {
        last_profile = t_end;
        Debug_Print(LOG_LEVEL_INFO, "ALC time: %lu ms\n", t_end - t_start);
    }
}

void Start_Levitation(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Starting levitation control...\r\n");

    memset(pid_controller.integral_pos,  0, sizeof(pid_controller.integral_pos));
    memset(pid_controller.prev_error_pos,0, sizeof(pid_controller.prev_error_pos));
    memset(pid_controller.output_pos,    0, sizeof(pid_controller.output_pos));
    memset(pid_controller.integral_ori,  0, sizeof(pid_controller.integral_ori));
    memset(pid_controller.prev_error_ori,0, sizeof(pid_controller.prev_error_ori));
    memset(pid_controller.output_ori,    0, sizeof(pid_controller.output_ori));

    system_state.levitation_active = 1;
    current_mode = MODE_LEVITATION;

    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (!sensors[i].is_connected) {
            Test_Sensor_Connection(i);
        }
    }

    Debug_Print(LOG_LEVEL_INFO, "Levitation started. Target: (%.1f, %.1f, %.1f)\r\n",
               pid_controller.setpoint_pos[0],
               pid_controller.setpoint_pos[1],
               pid_controller.setpoint_pos[2]);
}

void Stop_Levitation(void)
{
    Debug_Print(LOG_LEVEL_INFO, "Stopping levitation control...\r\n");
    Stop_All_Coils();
    system_state.levitation_active = 0;
    current_mode = MODE_IDLE;
}

void Set_Levitation_Target(float x, float y, float z)
{
    pid_controller.setpoint_pos[0] = x;
    pid_controller.setpoint_pos[1] = y;
    pid_controller.setpoint_pos[2] = z;
    Debug_Print(LOG_LEVEL_INFO, "Levitation target updated: (%.1f, %.1f, %.1f)\r\n", x, y, z);
}

void Get_Levitation_Status(char* buffer, uint16_t buffer_size)
{
    snprintf(buffer, buffer_size,
             "Levitation: %s\r\n"
             "Ball: X=%.1f, Y=%.1f, Z=%.1f\r\n"
             "Target: X=%.1f, Y=%.1f, Z=%.1f\r\n"
             "PID Out: X=%.3f, Y=%.3f, Z=%.3f\r\n",
             system_state.levitation_active ? "ACTIVE" : "INACTIVE",
             system_state.ball_position[0], system_state.ball_position[1], system_state.ball_position[2],
             pid_controller.setpoint_pos[0], pid_controller.setpoint_pos[1], pid_controller.setpoint_pos[2],
             pid_controller.output_pos[0], pid_controller.output_pos[1], pid_controller.output_pos[2]);
}

// Таблица калибровки тока
#define NUM_CALIB_POINTS 9
static const float calib_power[NUM_CALIB_POINTS] = {0.1f, 0.2f, 0.3f, 0.4f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
static const float calib_current[NUM_CALIB_POINTS] = {0.01f, 0.052f, 0.123f, 0.228f, 0.557f, 0.732f, 0.925f, 1.4f, 1.78f};

float PowerToCurrent(float power) {
    float abs_power = fabsf(power);
    float current = 0.0f;

    if (abs_power <= calib_power[0]) {
        current = calib_current[0] * (abs_power / calib_power[0]);
    }
    else if (abs_power >= calib_power[NUM_CALIB_POINTS-1]) {
        current = calib_current[NUM_CALIB_POINTS-1];
    }
    else {
        for (int i = 0; i < NUM_CALIB_POINTS - 1; i++) {
            if (abs_power >= calib_power[i] && abs_power <= calib_power[i+1]) {
                float t = (abs_power - calib_power[i]) / (calib_power[i+1] - calib_power[i]);
                current = calib_current[i] + t * (calib_current[i+1] - calib_current[i]);
                break;
            }
        }
    }
    return (power >= 0) ? current : -current;
}
