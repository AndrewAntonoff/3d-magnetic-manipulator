#include "json_commands.h"
#include "cJSON.h"
#include "debug_console.h"
#include "levitation_control.h"
#include "coil_driver.h"
#include <string.h>
extern PID_6DOF_t pid_controller;

void JSON_Init(void)
{
    // Ничего не требуется
}

void JSON_ProcessCommand(const char* json_string)
{
    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        Debug_Print(LOG_LEVEL_ERROR, "JSON parse error\n");
        return;
    }

    // Извлекаем поле "cmd" (обязательное)
    cJSON *cmd_item = cJSON_GetObjectItem(root, "cmd");
    if (cmd_item == NULL || !cJSON_IsString(cmd_item)) {
        Debug_Print(LOG_LEVEL_ERROR, "JSON: missing 'cmd' field\n");
        cJSON_Delete(root);
        return;
    }

    const char *cmd = cmd_item->valuestring;

    // --- Команда: установка коэффициентов ПИД ---
    if (strcmp(cmd, "set_pid") == 0) {
        cJSON *type = cJSON_GetObjectItem(root, "type"); // "pos" или "ori"
        cJSON *axis = cJSON_GetObjectItem(root, "axis"); // 0..2
        cJSON *kp = cJSON_GetObjectItem(root, "kp");
        cJSON *ki = cJSON_GetObjectItem(root, "ki");
        cJSON *kd = cJSON_GetObjectItem(root, "kd");

        if (type && cJSON_IsString(type) && axis && cJSON_IsNumber(axis) &&
            kp && cJSON_IsNumber(kp) && ki && cJSON_IsNumber(ki) && kd && cJSON_IsNumber(kd)) {
            int idx = axis->valueint;
            if (idx >= 0 && idx < 3) {
                if (strcmp(type->valuestring, "pos") == 0) {
                    pid_controller.Kp_pos[idx] = kp->valuedouble;
                    pid_controller.Ki_pos[idx] = ki->valuedouble;
                    pid_controller.Kd_pos[idx] = kd->valuedouble;
                    Debug_Print(LOG_LEVEL_INFO, "PID pos[%d] set\n", idx);
                } else if (strcmp(type->valuestring, "ori") == 0) {
                    pid_controller.Kp_ori[idx] = kp->valuedouble;
                    pid_controller.Ki_ori[idx] = ki->valuedouble;
                    pid_controller.Kd_ori[idx] = kd->valuedouble;
                    Debug_Print(LOG_LEVEL_INFO, "PID ori[%d] set\n", idx);
                }
            }
        }
    }

    // --- Команда: установка целевой позиции ---
    else if (strcmp(cmd, "set_target") == 0) {
        cJSON *x = cJSON_GetObjectItem(root, "x");
        cJSON *y = cJSON_GetObjectItem(root, "y");
        cJSON *z = cJSON_GetObjectItem(root, "z");
        if (x && y && z && cJSON_IsNumber(x) && cJSON_IsNumber(y) && cJSON_IsNumber(z)) {
            Set_Levitation_Target(x->valuedouble, y->valuedouble, z->valuedouble);
        }
    }

    // --- Команда: управление катушкой (тест) ---
    else if (strcmp(cmd, "coil") == 0) {
        cJSON *idx = cJSON_GetObjectItem(root, "idx");
        cJSON *power = cJSON_GetObjectItem(root, "power");
        if (idx && power && cJSON_IsNumber(idx) && cJSON_IsNumber(power)) {
            int i = idx->valueint;
            float p = power->valuedouble;
            if (i >= 0 && i < NUM_COILS) {
                Set_Coil_Power(i, p);
            }
        }
    }

    // --- Другие команды можно добавлять аналогично ---

    cJSON_Delete(root);
}
