#include "levitation_control.h"
#include "config.h"
#include "coil_driver.h"
#include "sensor_mlx90393.h"
#include "debug_console.h"
#include <math.h>
#include <string.h>

// Определяем глобальные переменные
PID_Controller_t pid_controller = {
    .Kp = {0.5f, 0.5f, 1.0f},
    .Ki = {0.05f, 0.05f, 0.1f},
    .Kd = {0.01f, 0.01f, 0.02f},
    .integral = {0, 0, 0},
    .prev_error = {0, 0, 0},
    .setpoint = {0, 0, 25.0f},  // Центр, высота 25мм
    .output = {0, 0, 0},
    .max_integral = 100.0f,
    .max_output = 1.0f
};

// Геометрия катушек (в мм)
static CoilGeometry_t coil_geometry[NUM_COILS];


void Initialize_Coil_Geometry(void) {
    // Верхний уровень (катушки 0-7)
    // Расположены по кругу радиусом 45мм, высота 25мм
    for(int i = 0; i < 8; i++) {
        float angle = (2.0f * M_PI * i) / 8.0f;
        coil_geometry[i].x = COIL_UPPER_RADIUS * cosf(angle);
        coil_geometry[i].y = COIL_UPPER_RADIUS * sinf(angle);
        coil_geometry[i].z = COIL_HEIGHT_LEVEL1;

        // Ориентация направлена к центру
        coil_geometry[i].orientation[0] = -cosf(angle);
        coil_geometry[i].orientation[1] = -sinf(angle);
        coil_geometry[i].orientation[2] = 0.1f;  // Немного вверх

        // Нормализуем вектор
        float length = sqrtf(
            coil_geometry[i].orientation[0] * coil_geometry[i].orientation[0] +
            coil_geometry[i].orientation[1] * coil_geometry[i].orientation[1] +
            coil_geometry[i].orientation[2] * coil_geometry[i].orientation[2]
        );

        coil_geometry[i].orientation[0] /= length;
        coil_geometry[i].orientation[1] /= length;
        coil_geometry[i].orientation[2] /= length;
    }

    // Нижний уровень (катушки 8-11)
    // Расположены по кругу радиусом 25мм, высота 5мм
    for(int i = 8; i < 12; i++) {
        float angle = (2.0f * M_PI * (i-8)) / 4.0f;
        coil_geometry[i].x = COIL_LOWER_RADIUS * cosf(angle);
        coil_geometry[i].y = COIL_LOWER_RADIUS * sinf(angle);
        coil_geometry[i].z = COIL_HEIGHT_LEVEL2;

        // Ориентация направлена к центру и вверх
        coil_geometry[i].orientation[0] = -cosf(angle);
        coil_geometry[i].orientation[1] = -sinf(angle);
        coil_geometry[i].orientation[2] = 0.5f;  // Сильнее вверх

        // Нормализуем вектор
        float length = sqrtf(
            coil_geometry[i].orientation[0] * coil_geometry[i].orientation[0] +
            coil_geometry[i].orientation[1] * coil_geometry[i].orientation[1] +
            coil_geometry[i].orientation[2] * coil_geometry[i].orientation[2]
        );

        coil_geometry[i].orientation[0] /= length;
        coil_geometry[i].orientation[1] /= length;
        coil_geometry[i].orientation[2] /= length;
    }
}

void Initialize_Sensor_Geometry(void) {
    // Датчики 0-3: по кругу на высоте 20мм, радиус 40мм
    for(int i = 0; i < 4; i++) {
        float angle = (2.0f * M_PI * i) / 4.0f;
        sensors[i].geometry.x = 40.0f * cosf(angle);
        sensors[i].geometry.y = 40.0f * sinf(angle);
        sensors[i].geometry.z = 20.0f;

        // Ориентация направлена к центру
        sensors[i].geometry.orientation[0] = -cosf(angle);
        sensors[i].geometry.orientation[1] = -sinf(angle);
        sensors[i].geometry.orientation[2] = 0.0f;
    }

    // Датчик 4: в центре на дне чаши
    sensors[4].geometry.x = 0.0f;
    sensors[4].geometry.y = 0.0f;
    sensors[4].geometry.z = 0.0f;
    sensors[4].geometry.orientation[0] = 0.0f;
    sensors[4].geometry.orientation[1] = 0.0f;
    sensors[4].geometry.orientation[2] = 1.0f;  // Смотрит вверх
}

void Calculate_Ball_Position(Position3D_t* position) {
    // Упрощенный алгоритм триангуляции по 5 датчикам
    // В реальности нужен более сложный алгоритм на основе магнитного диполя

    float weights[ACTIVE_SENSORS];
    float total_weight = 0.0f;

    // Рассчитываем веса на основе силы магнитного поля
    for(int i = 0; i < ACTIVE_SENSORS; i++) {
        if(sensors[i].is_connected) {
            float B = Calculate_Magnetic_Field_Strength(i);
            weights[i] = B * B;  // Квадрат для большего веса сильным сигналам
            total_weight += weights[i];
        } else {
            weights[i] = 0.0f;
        }
    }

    if(total_weight < 0.001f) {
        position->confidence = 0.0f;
        return;
    }

    // Взвешенная сумма позиций датчиков
    position->x = 0.0f;
    position->y = 0.0f;
    position->z = 0.0f;

    for(int i = 0; i < ACTIVE_SENSORS; i++) {
        if(weights[i] > 0) {
            // Предполагаем, что шар ближе к датчикам с большим полем
            float distance_factor = 1.0f / (weights[i] + 1.0f);

            position->x += sensors[i].geometry.x * distance_factor * weights[i];
            position->y += sensors[i].geometry.y * distance_factor * weights[i];
            position->z += sensors[i].geometry.z * distance_factor * weights[i];
        }
    }

    position->x /= total_weight;
    position->y /= total_weight;
    position->z /= total_weight;

    // Рассчитываем уверенность
    position->confidence = total_weight / (ACTIVE_SENSORS * 10000.0f); // Примерная нормализация
    if(position->confidence > 1.0f) position->confidence = 1.0f;

    position->timestamp = HAL_GetTick();
}

void Update_PID_Controller(float dt) {
    Position3D_t ball_pos;
    Calculate_Ball_Position(&ball_pos);

    if(ball_pos.confidence < 0.3f) {
        // Недостаточно данных для управления
        return;
    }

    // Обновляем системное состояние
    system_state.ball_position[0] = ball_pos.x;
    system_state.ball_position[1] = ball_pos.y;
    system_state.ball_position[2] = ball_pos.z;

    // Рассчитываем ошибку
    float error[3];
    error[0] = pid_controller.setpoint[0] - ball_pos.x;
    error[1] = pid_controller.setpoint[1] - ball_pos.y;
    error[2] = pid_controller.setpoint[2] - ball_pos.z;

    // Пропорциональная составляющая
    pid_controller.output[0] = pid_controller.Kp[0] * error[0];
    pid_controller.output[1] = pid_controller.Kp[1] * error[1];
    pid_controller.output[2] = pid_controller.Kp[2] * error[2];

    // Интегральная составляющая (с анти-виндъяпом)
    for(int i = 0; i < 3; i++) {
        pid_controller.integral[i] += error[i] * dt;

        // Ограничение интегральной составляющей
        if(pid_controller.integral[i] > pid_controller.max_integral) {
            pid_controller.integral[i] = pid_controller.max_integral;
        } else if(pid_controller.integral[i] < -pid_controller.max_integral) {
            pid_controller.integral[i] = -pid_controller.max_integral;
        }

        pid_controller.output[i] += pid_controller.Ki[i] * pid_controller.integral[i];
    }

    // Дифференциальная составляющая
    if(dt > 0.001f) {
        for(int i = 0; i < 3; i++) {
            float derivative = (error[i] - pid_controller.prev_error[i]) / dt;
            pid_controller.output[i] += pid_controller.Kd[i] * derivative;
            pid_controller.prev_error[i] = error[i];
        }
    }

    // Ограничение выходного сигнала
    for(int i = 0; i < 3; i++) {
        if(pid_controller.output[i] > pid_controller.max_output) {
            pid_controller.output[i] = pid_controller.max_output;
        } else if(pid_controller.output[i] < -pid_controller.max_output) {
            pid_controller.output[i] = -pid_controller.max_output;
        }
    }
}

void Calculate_Coil_Forces(float fx, float fy, float fz, float* coil_powers) {
    // Распределяем силы по катушкам на основе их ориентации

    for(int i = 0; i < NUM_COILS; i++) {
        // Проекция силы на ориентацию катушки
        float projection =
            fx * coil_geometry[i].orientation[0] +
            fy * coil_geometry[i].orientation[1] +
            fz * coil_geometry[i].orientation[2];

        // Также учитываем расстояние до центра
        float distance = sqrtf(
            coil_geometry[i].x * coil_geometry[i].x +
            coil_geometry[i].y * coil_geometry[i].y +
            coil_geometry[i].z * coil_geometry[i].z
        );

        // Коэффициент эффективности (катушки ближе к центру более эффективны)
        float efficiency = 1.0f / (distance + 1.0f);

        coil_powers[i] = projection * efficiency * 2.0f; // Усиливаем

        // Ограничение
        if(coil_powers[i] > 1.0f) coil_powers[i] = 1.0f;
        if(coil_powers[i] < -1.0f) coil_powers[i] = -1.0f;
    }

    // Нормализуем, чтобы суммарная мощность не превышала лимит
    float max_power = 0.0f;
    for(int i = 0; i < NUM_COILS; i++) {
        if(fabsf(coil_powers[i]) > max_power) {
            max_power = fabsf(coil_powers[i]);
        }
    }

    if(max_power > 0.8f) {
        float scale = 0.8f / max_power;
        for(int i = 0; i < NUM_COILS; i++) {
            coil_powers[i] *= scale;
        }
    }
}

void Apply_Levitation_Control(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    if(last_time == 0) {
        last_time = current_time;
        return;
    }

    float dt = (current_time - last_time) / 1000.0f;  // в секундах
    if(dt < 0.001f) return;  // Слишком маленький интервал

    // Обновляем ПИД контроллер
    Update_PID_Controller(dt);

    // Преобразуем выход ПИД в силы
    float fx = pid_controller.output[0] * 10.0f;  // Масштабируем
    float fy = pid_controller.output[1] * 10.0f;
    float fz = pid_controller.output[2] * 10.0f;

    // Добавляем гравитационную компенсацию (примерно 9.8 м/с²)
    fz += 9.8f;

    // Рассчитываем мощности катушек
    float coil_powers[NUM_COILS];
    Calculate_Coil_Forces(fx, fy, fz, coil_powers);

    // Применяем мощности к катушкам
    for(int i = 0; i < NUM_COILS; i++) {
        Set_Coil_Power(i, coil_powers[i]);
    }

    last_time = current_time;
}

void Start_Levitation(void) {
    Debug_Print(LOG_LEVEL_INFO, "Starting levitation control...\r\n");

    // Сбрасываем ПИД контроллер
    memset(pid_controller.integral, 0, sizeof(pid_controller.integral));
    memset(pid_controller.prev_error, 0, sizeof(pid_controller.prev_error));
    memset(pid_controller.output, 0, sizeof(pid_controller.output));

    // Устанавливаем режим
    system_state.levitation_active = 1;
    current_mode = MODE_LEVITATION;

    // Включаем все датчики
    for(int i = 0; i < ACTIVE_SENSORS; i++) {
        if(!sensors[i].is_connected) {
            Test_Sensor_Connection(i);
        }
    }

    Debug_Print(LOG_LEVEL_INFO, "Levitation started. Target: (%.1f, %.1f, %.1f)\r\n",
               pid_controller.setpoint[0],
               pid_controller.setpoint[1],
               pid_controller.setpoint[2]);
}

void Stop_Levitation(void) {
    Debug_Print(LOG_LEVEL_INFO, "Stopping levitation control...\r\n");

    // Выключаем все катушки
    Stop_All_Coils();

    // Сбрасываем флаги
    system_state.levitation_active = 0;
    current_mode = MODE_IDLE;
}

void Set_Levitation_Target(float x, float y, float z) {
    pid_controller.setpoint[0] = x;
    pid_controller.setpoint[1] = y;
    pid_controller.setpoint[2] = z;

    Debug_Print(LOG_LEVEL_INFO, "Levitation target updated: (%.1f, %.1f, %.1f)\r\n", x, y, z);
}

void Get_Levitation_Status(char* buffer, uint16_t buffer_size) {
    Position3D_t pos;
    Calculate_Ball_Position(&pos);

    snprintf(buffer, buffer_size,
             "Levitation: %s\r\n"
             "Ball: X=%.1f, Y=%.1f, Z=%.1f (conf=%.2f)\r\n"
             "Target: X=%.1f, Y=%.1f, Z=%.1f\r\n"
             "PID Out: X=%.3f, Y=%.3f, Z=%.3f\r\n",
             system_state.levitation_active ? "ACTIVE" : "INACTIVE",
             pos.x, pos.y, pos.z, pos.confidence,
             pid_controller.setpoint[0], pid_controller.setpoint[1], pid_controller.setpoint[2],
             pid_controller.output[0], pid_controller.output[1], pid_controller.output[2]);
}
