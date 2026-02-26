#include "levitation_control.h"
#include "config.h"
#include "main.h"          // для доступа к last_imu_data, imu_packet_ready
#include "coil_driver.h"
#include "sensor_mlx90393.h"
#include "debug_console.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Если нужно, можно раскомментировать после добавления CMSIS-DSP
// #include "arm_math.h"

extern MLX90393_t sensors[NUM_SENSORS];

// ------------------------------------------------------------------
// Константы
// ------------------------------------------------------------------
#define MU0_4PI 1e-7f                     // μ0/(4π) в СИ
#define HOME_Z          25.0f              // домашняя высота (мм)
#define RELEASE_DIST_THRESH  2.0f          // порог расстояния для отпускания (мм)
#define RELEASE_SPEED_THRESH 5.0f          // порог скорости (мм/с)

// ------------------------------------------------------------------
// Глобальные переменные, определённые здесь
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

// ------------------------------------------------------------------
// Вспомогательные функции (математика поля)
// ------------------------------------------------------------------

/**
 * Вычисляет магнитное поле диполя с моментом m[3], расположенного в p[3],
 * в точке s[3]. Результат в B[3].
 */
static void DipoleField(const float p[3], const float m[3], const float s[3], float B[3]) {
    float r[3] = { s[0] - p[0], s[1] - p[1], s[2] - p[2] };  // Нет /1000; уже в метрах
    float r2 = r[0]*r[0] + r[1]*r[1] + r[2]*r[2];
    float r_norm = sqrtf(r2);
    if (r_norm < 1e-6f) {
        B[0] = B[1] = B[2] = 0;
        return;
    }
    float r3 = r2 * r_norm;
    float r5 = r3 * r2;
    float mr = m[0]*r[0] + m[1]*r[1] + m[2]*r[2];
    float factor1 = 3.0f * mr / r5;
    float factor2 = 1.0f / r3;
    B[0] = MU0_4PI * (factor1 * r[0] - factor2 * m[0]);
    B[1] = MU0_4PI * (factor1 * r[1] - factor2 * m[1]);
    B[2] = MU0_4PI * (factor1 * r[2] - factor2 * m[2]);
}

/**
 * Вычисляет силу и момент, действующие на диполь шара со стороны одной катушки.
 */
static void Compute_ForceTorque(uint8_t coil_idx, const float ball_pos[3],
                                const float ball_moment[3],
                                float force[3], float torque[3])
{
    // Координаты катушки (в мм)
    float r_c[3] = {coil_geometry[coil_idx].x,
                    coil_geometry[coil_idx].y,
                    coil_geometry[coil_idx].z};
    // Вектор от катушки к шару (переводим в метры)
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

    // Скалярные произведения
    float Mr = ball_moment[0]*r[0] + ball_moment[1]*r[1] + ball_moment[2]*r[2];
    float mr = coil_geometry[coil_idx].orientation[0]*r[0] +
               coil_geometry[coil_idx].orientation[1]*r[1] +
               coil_geometry[coil_idx].orientation[2]*r[2];
    float Mm = ball_moment[0]*coil_geometry[coil_idx].orientation[0] +
               ball_moment[1]*coil_geometry[coil_idx].orientation[1] +
               ball_moment[2]*coil_geometry[coil_idx].orientation[2];

    // Сила (без коэффициента mu0/(4pi))
    float factor = 3.0f / r5;
    for (int i = 0; i < 3; i++) {
        force[i] = factor * ( Mr * coil_geometry[coil_idx].orientation[i] +
                              mr * ball_moment[i] +
                              Mm * r[i] -
                              5.0f * Mr * mr * r[i] / r2 );
    }

    // Поле катушки в точке шара (для момента)
    float B[3];
    float r3 = r2 * r_norm;
    factor = 1.0f / r3;
    float coeff = 3.0f * mr / r2;
    for (int i = 0; i < 3; i++) {
        B[i] = factor * (coeff * r[i] - coil_geometry[coil_idx].orientation[i]);
    }

    // Момент силы T = M × B
    torque[0] = ball_moment[1]*B[2] - ball_moment[2]*B[1];
    torque[1] = ball_moment[2]*B[0] - ball_moment[0]*B[2];
    torque[2] = ball_moment[0]*B[1] - ball_moment[1]*B[0];

    // Умножим на mu0/(4pi)
    for (int i = 0; i < 3; i++) {
        force[i]  *= MU0_4PI;
        torque[i] *= MU0_4PI;
    }
}

/**
 * Заполнение матрицы A (6x12) для текущего положения и момента шара.
 */
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
    // Отладка: среднее значение матрицы
        float sum = 0.0f;
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 12; j++)
                sum += fabsf(A[i][j]);
        Debug_Print(LOG_LEVEL_INFO, "Matrix A avg = %.2e\n", sum / (6*12));
}

/**
 * Обращение матрицы 6x6 методом Гаусса-Жордана.
 * Возвращает 1 при успехе, 0 если матрица вырождена.
 */
static int InvertMatrix_6x6(const float A[6][6], float inv[6][6])
{
    float aug[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            aug[i][j] = A[i][j];
            aug[i][j+6] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Прямой ход Гаусса
    for (int col = 0; col < 6; col++) {
        // Поиск главного элемента
        int pivot = col;
        float max_val = fabsf(aug[col][col]);
        for (int row = col+1; row < 6; row++) {
            if (fabsf(aug[row][col]) > max_val) {
                max_val = fabsf(aug[row][col]);
                pivot = row;
            }
        }
        if (max_val < 1e-6f) return 0;

        // Обмен строк
        if (pivot != col) {
            for (int j = 0; j < 12; j++) {
                float tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }

        // Нормализация строки
        float div = aug[col][col];
        for (int j = 0; j < 12; j++) {
            aug[col][j] /= div;
        }

        // Обнуление остальных строк
        for (int row = 0; row < 6; row++) {
            if (row != col) {
                float factor = aug[row][col];
                for (int j = 0; j < 12; j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }
    }

    // Извлечение обратной матрицы
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            inv[i][j] = aug[i][j+6];
        }
    }
    return 1;
}

/**
 * Решение системы I = pinv(A) * u с регуляризацией.
 */
static void Solve_Currents(const float A[6][12], const float u[6],
                           float I[12], float lambda)
{
    // C = A * A^T  (6x6)
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

    // Регуляризация
    for (int i = 0; i < 6; i++) {
        C[i][i] += lambda * lambda;
    }

    // Обращение C
    float C_inv[6][6];
    if (!InvertMatrix_6x6(C, C_inv)) {
    	Debug_Print(LOG_LEVEL_ERROR, "Solve_Currents: Matrix inversion failed!\n");
        memset(I, 0, NUM_COILS * sizeof(float));
        return;
    }

    // tmp = C_inv * u
    float tmp[6];
    for (int i = 0; i < 6; i++) {
        tmp[i] = 0;
        for (int j = 0; j < 6; j++) {
            tmp[i] += C_inv[i][j] * u[j];
        }
    }

    // I = A^T * tmp
    for (int k = 0; k < NUM_COILS; k++) {
        I[k] = 0;
        for (int i = 0; i < 6; i++) {
            I[k] += A[i][k] * tmp[i];
        }
    }
    // Ограничение токов: масштабируем, если любой превышает 1.0
        float max_abs = 0.0f;
        for (int k = 0; k < NUM_COILS; k++) {
            float abs_val = fabsf(I[k]);
            if (abs_val > max_abs) max_abs = abs_val;
        }
        if (max_abs > 1.0f) {
            float scale = 1.0f / max_abs;
            for (int k = 0; k < NUM_COILS; k++) {
                I[k] *= scale;
            }
            Debug_Print(LOG_LEVEL_INFO, "Solve_Currents: scaled by %.3f\n", scale);
        }

    // Отладка: выводим первые 3 тока и их среднее
        Debug_Print(LOG_LEVEL_INFO, "Solve_Currents: I0=%.3f I1=%.3f I2=%.3f\n", I[0], I[1], I[2]);
        if (!InvertMatrix_6x6(C, C_inv)) {
            Debug_Print(LOG_LEVEL_ERROR, "Matrix inversion failed!\n");
            memset(I, 0, NUM_COILS * sizeof(float));
            return;
        }
}

/**
 * Вычисляет вектор невязки (разность между моделью и измерением)
 * для текущей позиции p и ориентации (через m).
 * residual – массив из 15 чисел (сначала x,y,z для датчика 0, потом датчик 1 и т.д.)
 */
static void ComputeResidual(const float p[3], const float m[3],
                            const float ball_meas[5][3],
                            float residual[15])
{
    for (int i = 0; i < 5; i++) {
        float s[3] = {sensors[i].geometry.x / 1000.0f,
                      sensors[i].geometry.y / 1000.0f,
                      sensors[i].geometry.z / 1000.0f};
        float B_model[3];
        DipoleField(p, m, s, B_model);
        residual[3*i + 0] = B_model[0] - ball_meas[i][0];
        residual[3*i + 1] = B_model[1] - ball_meas[i][1];
        residual[3*i + 2] = B_model[2] - ball_meas[i][2];

        // Отладка: выводим для первого датчика на первой итерации (можно добавить условие)
        if (i == 0) {
            Debug_Print(LOG_LEVEL_INFO, "Residual: B_model[0]=%.3e, ball_meas[0]=%.3e\n",
                        B_model[0], ball_meas[i][0]);
        }
    }
}

/**
 * Численное вычисление якобиана (15×3) методом конечных разностей.
 * Заполняет матрицу J (15 строк, 3 столбца).
 */
static void ComputeJacobian(const float p[3], const float m[3],
                            const float ball_meas[5][3],
                            float J[15][3])
{
	const float delta = 1e-6f;  // Меньший шаг (1 мкм) для стабильности
    float residual0[15];
    ComputeResidual(p, m, ball_meas, residual0);

    for (int j = 0; j < 3; j++) {
        float p_plus[3] = { p[0], p[1], p[2] };
        p_plus[j] += delta;
        float res_plus[15];
        ComputeResidual(p_plus, m, ball_meas, res_plus);

        for (int i = 0; i < 15; i++) {
            J[i][j] = (res_plus[i] - residual0[i]) / delta;
        }
    }
}

/**
 * Одна итерация Гаусса–Ньютона для уточнения позиции.
 */
static void GaussNewtonIteration(float p[3], const float m[3],
                                 const float ball_meas[5][3])
{
    float J[15][3];
    float r[15];

    ComputeResidual(p, m, ball_meas, r);
    ComputeJacobian(p, m, ball_meas, J);

    // Вычисляем JᵀJ (3×3) и Jᵀr (3×1)
    float JTJ[3][3] = {{0}};
    float JTr[3] = {0};

    for (int i = 0; i < 15; i++) {
        for (int k = 0; k < 3; k++) {
            JTr[k] += J[i][k] * r[i];
            for (int l = 0; l < 3; l++) {
                JTJ[k][l] += J[i][k] * J[i][l];
            }
        }
    }

    // Добавляем демпфирование Левенберга-Марквардта для стабильности
    float lambda = 1e-3f;  // Начальный коэффициент демпфирования (подстройте по тестам)
    for (int k = 0; k < 3; k++) {
        JTJ[k][k] += lambda * JTJ[k][k];  // Демпфируем диагональ
    }

    // Вычисляем детерминант (3x3 матрица)
    float det = JTJ[0][0] * (JTJ[1][1]*JTJ[2][2] - JTJ[1][2]*JTJ[2][1])
              - JTJ[0][1] * (JTJ[1][0]*JTJ[2][2] - JTJ[1][2]*JTJ[2][0])
              + JTJ[0][2] * (JTJ[1][0]*JTJ[2][1] - JTJ[1][1]*JTJ[2][0]);

    Debug_Print(LOG_LEVEL_INFO, "GN: det=%.2e, JTJ[0][0]=%.2e, JTr=(%.2e,%.2e,%.2e)\n",
                det, JTJ[0][0], JTr[0], JTr[1], JTr[2]);

    if (fabsf(det) < 1e-12f) {
        Debug_Print(LOG_LEVEL_INFO, "GN: singular, increasing lambda and retrying\n");
        lambda *= 10.0f;  // Увеличиваем демпфирование при вырождении
        for (int k = 0; k < 3; k++) {
            JTJ[k][k] += lambda * JTJ[k][k];  // Повторно демпфируем
        }
        // Пересчитываем det (опционально: можно добавить цикл для нескольких попыток)
        det = JTJ[0][0] * (JTJ[1][1]*JTJ[2][2] - JTJ[1][2]*JTJ[2][1])
            - JTJ[0][1] * (JTJ[1][0]*JTJ[2][2] - JTJ[1][2]*JTJ[2][0])
            + JTJ[0][2] * (JTJ[1][0]*JTJ[2][1] - JTJ[1][1]*JTJ[2][0]);
        if (fabsf(det) < 1e-12f) {
            Debug_Print(LOG_LEVEL_INFO, "GN: still singular, skipping\n");
            return;
        }
    }

    // Решаем систему JTJ * Δp = -JTr (используем обратную матрицу для 3x3)
    // Вычисляем обратную JTJ (формулы для 3x3)
    float inv_JTJ[3][3];
    float inv_det = 1.0f / det;

    inv_JTJ[0][0] = inv_det * (JTJ[1][1]*JTJ[2][2] - JTJ[1][2]*JTJ[2][1]);
    inv_JTJ[0][1] = inv_det * (JTJ[0][2]*JTJ[2][1] - JTJ[0][1]*JTJ[2][2]);
    inv_JTJ[0][2] = inv_det * (JTJ[0][1]*JTJ[1][2] - JTJ[0][2]*JTJ[1][1]);

    inv_JTJ[1][0] = inv_det * (JTJ[1][2]*JTJ[2][0] - JTJ[1][0]*JTJ[2][2]);
    inv_JTJ[1][1] = inv_det * (JTJ[0][0]*JTJ[2][2] - JTJ[0][2]*JTJ[2][0]);
    inv_JTJ[1][2] = inv_det * (JTJ[0][2]*JTJ[1][0] - JTJ[0][0]*JTJ[1][2]);

    inv_JTJ[2][0] = inv_det * (JTJ[1][0]*JTJ[2][1] - JTJ[1][1]*JTJ[2][0]);
    inv_JTJ[2][1] = inv_det * (JTJ[0][1]*JTJ[2][0] - JTJ[0][0]*JTJ[2][1]);
    inv_JTJ[2][2] = inv_det * (JTJ[0][0]*JTJ[1][1] - JTJ[0][1]*JTJ[1][0]);

    // Δp = inv_JTJ * (-JTr)
    float neg_JTr[3] = {-JTr[0], -JTr[1], -JTr[2]};
    float dp[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dp[i] += inv_JTJ[i][j] * neg_JTr[j];
        }
    }

    // Обновляем позицию
    p[0] += dp[0];
    p[1] += dp[1];
    p[2] += dp[2];

    // Отладка (без prev_p, но с выводом dp и новой p)
    Debug_Print(LOG_LEVEL_INFO, "GN iter: dp=(%.3f,%.3f,%.3f) -> p=(%.3f,%.3f,%.3f)\n",
                dp[0], dp[1], dp[2], p[0], p[1], p[2]);
}

// ------------------------------------------------------------------
// Основные функции управления
// ------------------------------------------------------------------

void Initialize_Coil_Geometry(void) {
    // Нижний уровень: 4 катушки на z=5 мм, r=25 мм (диаметр 50 мм /2), на 90° друг от друга
    float lower_r = 25.0f;  // мм
    float lower_z = 5.0f;
    for (int i = 0; i < 4; i++) {
        float angle = i * (M_PI / 2.0f);  // 0°,90°,180°,270°
        coil_geometry[i].x = lower_r * cosf(angle);
        coil_geometry[i].y = lower_r * sinf(angle);
        coil_geometry[i].z = lower_z;
        // Ориентация: направлены к центру (0,0,20 мм), нормализуйте
        float dx = -coil_geometry[i].x, dy = -coil_geometry[i].y, dz = 20.0f - lower_z;
        float norm = sqrtf(dx*dx + dy*dy + dz*dz);
        coil_geometry[i].orientation[0] = dx / norm;
        coil_geometry[i].orientation[1] = dy / norm;
        coil_geometry[i].orientation[2] = dz / norm;
    }
    // Верхний уровень: 8 катушек на z=25 мм, r=45 мм (диаметр 90 мм /2), на 45° друг от друга
    float upper_r = 45.0f;
    float upper_z = 25.0f;
    for (int i = 4; i < 12; i++) {
        float angle = (i-4) * (M_PI / 4.0f);
        coil_geometry[i].x = upper_r * cosf(angle);
        coil_geometry[i].y = upper_r * sinf(angle);
        coil_geometry[i].z = upper_z;
        float dx = -coil_geometry[i].x, dy = -coil_geometry[i].y, dz = 20.0f - upper_z;  // Центр на ~20 мм высоте
        float norm = sqrtf(dx*dx + dy*dy + dz*dz);
        coil_geometry[i].orientation[0] = dx / norm;
        coil_geometry[i].orientation[1] = dy / norm;
        coil_geometry[i].orientation[2] = dz / norm;
    }
}

void Initialize_Sensor_Geometry(void) {
    // Центр снизу: (0,0,0)
    sensors[0].geometry.x = 0.0f;
    sensors[0].geometry.y = 0.0f;
    sensors[0].geometry.z = 0.0f;
    // 4 вокруг, на z~20 мм (выше дна), r=55 мм (диаметр чаши ~110 мм /2), на 90° друг от друга
    float sensor_r = 55.0f;
    float sensor_z = 20.0f;  // Подстройте по точному размещению
    for (int i = 1; i < 5; i++) {
        float angle = (i-1) * (M_PI / 2.0f);
        sensors[i].geometry.x = sensor_r * cosf(angle);
        sensors[i].geometry.y = sensor_r * sinf(angle);
        sensors[i].geometry.z = sensor_z;
    }
}

void Calculate_Ball_Position(Position3D_t* position)
{
    float weights[ACTIVE_SENSORS];
    float total_weight = 0.0f;

    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        if (sensors[i].is_connected) {
            float B = sqrtf(sensors[i].magnetic_field[0]*sensors[i].magnetic_field[0] +
                            sensors[i].magnetic_field[1]*sensors[i].magnetic_field[1] +
                            sensors[i].magnetic_field[2]*sensors[i].magnetic_field[2]);
            weights[i] = B * B; // квадрат модуля
            total_weight += weights[i];
        } else {
            weights[i] = 0.0f;
        }
    }

    if (total_weight < 0.001f) {
        position->x = 0; position->y = 0; position->z = 0;
        position->confidence = 0.0f;
        return;
    }

    position->x = 0.0f; position->y = 0.0f; position->z = 0.0f;
    for (int i = 0; i < ACTIVE_SENSORS; i++) {
        position->x += sensors[i].geometry.x * weights[i];
        position->y += sensors[i].geometry.y * weights[i];
        position->z += sensors[i].geometry.z * weights[i];
    }
    position->x /= total_weight;
    position->y /= total_weight;
    position->z /= total_weight;

    // Примерная уверенность (нормализованная)
    position->confidence = total_weight / (ACTIVE_SENSORS * 10000.0f);
    if (position->confidence > 1.0f) position->confidence = 1.0f;
    position->timestamp = HAL_GetTick();

    Debug_Print(LOG_LEVEL_INFO, "Rough position: (%.1f,%.1f,%.1f) conf=%.2f\n",
                position->x, position->y, position->z, position->confidence);
}

// ------------------------------------------------------------------
// ПИД-регулятор (6 степеней свободы)
// ------------------------------------------------------------------
PID_6DOF_t pid_controller = {
    .Kp_pos      = {0.5f, 0.5f, 1.0f},
    .Ki_pos      = {0.05f, 0.05f, 0.1f},
    .Kd_pos      = {0.01f, 0.01f, 0.02f},
    .integral_pos= {0,0,0},
    .prev_error_pos={0,0,0},
    .setpoint_pos= {0,0,25.0f},
    .output_pos  = {0,0,0},

    .Kp_ori      = {0.1f, 0.1f, 0.05f},
    .Ki_ori      = {0.01f, 0.01f, 0.005f},
    .Kd_ori      = {0.05f, 0.05f, 0.02f},
    .integral_ori= {0,0,0},
    .prev_error_ori={0,0,0},
    .setpoint_ori= {0,0,0},
    .output_ori  = {0,0,0},

    .max_integral = 100.0f,
    .max_output   = 1.0f
};

void Update_PID_Controller(float dt)
{
    Position3D_t ball_pos;
    Calculate_Ball_Position(&ball_pos);
    system_state.ball_position[0] = ball_pos.x;
    system_state.ball_position[1] = ball_pos.y;
    system_state.ball_position[2] = ball_pos.z;

    // Получаем данные IMU
    IMU_Data_t imu;
    __disable_irq();
    memcpy(&imu, (void*)&last_imu_data, sizeof(IMU_Data_t));
    __enable_irq();

    // Преобразуем в физические единицы
    float roll  = imu.roll  * 0.001f;      // радианы
    float pitch = imu.pitch * 0.001f;
    float yaw   = imu.yaw   * 0.001f;

    float gyro_x = imu.gyro_x * 0.001f * (M_PI/180.0f); // mdps -> rad/s
    float gyro_y = imu.gyro_y * 0.001f * (M_PI/180.0f);
    float gyro_z = imu.gyro_z * 0.001f * (M_PI/180.0f);

    // Отладочный вывод IMU раз в секунду
    static uint32_t last_imu_debug = 0;
    if (HAL_GetTick() - last_imu_debug > 1000) {
        last_imu_debug = HAL_GetTick();
        Debug_Print(LOG_LEVEL_INFO, "IMU angles: roll=%.2f pitch=%.2f yaw=%.2f\n",
                    roll, pitch, yaw);
    }

    // Ошибка по позиции
    float error_pos[3] = {
        pid_controller.setpoint_pos[0] - ball_pos.x,
        pid_controller.setpoint_pos[1] - ball_pos.y,
        pid_controller.setpoint_pos[2] - ball_pos.z
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

    // Ошибка по ориентации
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

    // Пока считаем момент шара постоянным вертикальным
    float ball_moment[3] = {0.0f, 0.0f, 1.0f}; // временно большой момент

    float A[6][12];
    Build_Matrix_A(ball_pos, ball_moment, A);

    float u[6] = {fx, fy, fz, tx, ty, tz};
    float I[12];
    Solve_Currents(A, u, I, 0.001f); // lambda = 0.001

    for (int i = 0; i < NUM_COILS; i++) {
        coil_powers[i] = I[i];
    }
}

void Apply_Levitation_Control(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (last_time == 0) { last_time = current_time; return; }
    float dt = (current_time - last_time) / 1000.0f;
    if (dt < 0.001f) return;

    Update_PID_Controller(dt);

    float fx = pid_controller.output_pos[0] * 10.0f;
    float fy = pid_controller.output_pos[1] * 10.0f;
    float fz = pid_controller.output_pos[2] * 10.0f + 9.8f; // компенсация гравитации

    float tx = pid_controller.output_ori[0] * 1.0f;
    float ty = pid_controller.output_ori[1] * 1.0f;
    float tz = pid_controller.output_ori[2] * 1.0f;

    float coil_powers[NUM_COILS];
    Calculate_Coil_Forces(fx, fy, fz, tx, ty, tz, coil_powers);

    for (int i = 0; i < NUM_COILS; i++) {
        Set_Coil_Power(i, coil_powers[i]);
    }

    last_time = current_time;
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
    Position3D_t pos;
    Calculate_Ball_Position(&pos);
    snprintf(buffer, buffer_size,
             "Levitation: %s\r\n"
             "Ball: X=%.1f, Y=%.1f, Z=%.1f (conf=%.2f)\r\n"
             "Target: X=%.1f, Y=%.1f, Z=%.1f\r\n"
             "PID Out: X=%.3f, Y=%.3f, Z=%.3f\r\n",
             system_state.levitation_active ? "ACTIVE" : "INACTIVE",
             pos.x, pos.y, pos.z, pos.confidence,
             pid_controller.setpoint_pos[0], pid_controller.setpoint_pos[1], pid_controller.setpoint_pos[2],
             pid_controller.output_pos[0], pid_controller.output_pos[1], pid_controller.output_pos[2]);
}

void EstimateBallPosition(float ball_pos[3], const float orientation[4])
{
    // 1. Грубая оценка (уже в мм)
    Position3D_t rough;
    Calculate_Ball_Position(&rough);
    float p_m[3]; // позиция в метрах

    if (rough.confidence > 0.1f) {
        p_m[0] = rough.x / 1000.0f;
        p_m[1] = rough.y / 1000.0f;
        p_m[2] = rough.z / 1000.0f;
    } else {
        p_m[0] = ball_pos[0] / 1000.0f;
        p_m[1] = ball_pos[1] / 1000.0f;
        p_m[2] = ball_pos[2] / 1000.0f;
        if (p_m[0] == 0 && p_m[1] == 0 && p_m[2] == 0) {
            p_m[2] = 0.025f; // центр чаши в метрах (25 мм)
        }
    }

    Debug_Print(LOG_LEVEL_INFO, "EstimateBallPosition: rough_m=(%.3f,%.3f,%.3f)\n",
                p_m[0], p_m[1], p_m[2]);

    // 2. Измерения с датчиков (оставляем в µT, они не требуют пересчёта)
    float B_ball_T[5][3];
    for (int i = 0; i < 5; i++) {
        B_ball_T[i][0] = sensors[i].magnetic_field[0] * 1e-6f;
        B_ball_T[i][1] = sensors[i].magnetic_field[1] * 1e-6f;
        B_ball_T[i][2] = sensors[i].magnetic_field[2] * 1e-6f;
    }

    // 3. Магнитный момент шара (пока постоянный, в системе СИ)
    float m[3] = {0.0f, 0.0f, 1.0f};  // Увеличьте до ~1.0 А·м²; калибруйте эмпирически
    // 4. Итерации Гаусса-Ньютона (p_m уже в метрах)
    for (int iter = 0; iter < 5; iter++) {
        GaussNewtonIteration(p_m, m, B_ball_T);   // было B_ball, исправлено на B_ball_T
    }

    // 5. Ограничение в пределах чаши (переводим в мм для удобства)
    float p_mm[3] = {p_m[0]*1000, p_m[1]*1000, p_m[2]*1000};
    if (p_mm[2] < 0.0f) p_mm[2] = 0.0f;
    if (p_mm[2] > 40.0f) p_mm[2] = 40.0f;
    float xy_lim = 55.0f;
    float xy_dist = sqrtf(p_mm[0]*p_mm[0] + p_mm[1]*p_mm[1]);
    if (xy_dist > xy_lim) {
        float scale = xy_lim / xy_dist;
        p_mm[0] *= scale;
        p_mm[1] *= scale;
    }

    ball_pos[0] = p_mm[0];
    ball_pos[1] = p_mm[1];
    ball_pos[2] = p_mm[2];

    Debug_Print(LOG_LEVEL_INFO, "EstimateBallPosition: final=(%.1f,%.1f,%.1f)\n",
                ball_pos[0], ball_pos[1], ball_pos[2]);
}

static uint8_t IsBallReleased(void)
{
    // Функция пока не используется, но для полноты оставим заглушку
    static float prev_pos[3] = {0};
    float dt = 0.01f; // примерное dt, лучше передавать параметром

    float vel_x = (system_state.ball_position[0] - prev_pos[0]) / dt;
    float vel_y = (system_state.ball_position[1] - prev_pos[1]) / dt;
    float vel_z = (system_state.ball_position[2] - prev_pos[2]) / dt;
    float speed = sqrtf(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z);

    float dist_to_home = sqrtf( system_state.ball_position[0]*system_state.ball_position[0] +
                                system_state.ball_position[1]*system_state.ball_position[1] +
                                (system_state.ball_position[2]-HOME_Z)*(system_state.ball_position[2]-HOME_Z) );

    if (dist_to_home < RELEASE_DIST_THRESH && speed < RELEASE_SPEED_THRESH) {
        return 1;
    }
    return 0;
}
