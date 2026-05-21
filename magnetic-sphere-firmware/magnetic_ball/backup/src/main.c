#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <math.h>

// Определяем M_PI вручную (picolibc не включает его по умолчанию)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// UUID для кастомного сервиса Magnetic Ball
#define BT_UUID_MAGBALL_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_MAGBALL_DATA_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_MAGBALL_CMD_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

static const struct bt_uuid_128 magball_service_uuid = BT_UUID_INIT_128(
    BT_UUID_MAGBALL_VAL);
static const struct bt_uuid_128 magball_data_uuid = BT_UUID_INIT_128(
    BT_UUID_MAGBALL_DATA_VAL);
static const struct bt_uuid_128 magball_cmd_uuid = BT_UUID_INIT_128(
    BT_UUID_MAGBALL_CMD_VAL);

// Структура для данных IMU
#pragma pack(push, 1)
struct sensor_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float quaternion[4];     // w, x, y, z
    uint32_t timestamp;
    uint8_t sequence;
    uint8_t battery;
    uint8_t imu_status;      // 0=нет IMU, 1=IMU работает
};
#pragma pack(pop)

// Глобальные переменные
static struct sensor_data current_data;
static uint32_t sample_count = 0;
static bool ble_connected = false;
static uint8_t led_state = 0;
static uint32_t data_interval_ms = 100;

// Mahony filter variables
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Quaternion: w, x, y, z
static float b[3] = {0.0f, 0.0f, 0.0f};       // Gyro bias
static const float kP = 1.0f;                  // Proportional gain
static const float kI = 0.3f;                  // Integral gain
static const float dt = 0.01f;                 // Sample time (100 Hz)

// Калибровка
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static const float accel_scale[3] = {1.0f, 1.0f, 1.0f};  // Можно расширить позже

// I2C устройство
static const struct device *i2c_dev;

// LSM6DS3 регистры
#define LSM6DS3_ADDR 0x6B
#define LSM6DS3_WHO_AM_I 0x0F
#define LSM6DS3_CTRL1_XL 0x10
#define LSM6DS3_CTRL2_G 0x11
#define LSM6DS3_OUTX_L_XL 0x28

// Вспомогательные I2C функции
static int i2c_write_reg(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    struct i2c_msg msg = {
        .buf = buf,
        .len = 2,
        .flags = I2C_MSG_WRITE | I2C_MSG_STOP,
    };
    return i2c_transfer(dev, &msg, 1, addr);
}

static int i2c_read_reg(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t *value) {
    struct i2c_msg msgs[2];
    msgs[0].buf = &reg;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;
    msgs[1].buf = value;
    msgs[1].len = 1;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;
    return i2c_transfer(dev, msgs, 2, addr);
}

static int i2c_read_multi(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    struct i2c_msg msgs[2];
    msgs[0].buf = &reg;
    msgs[0].len = 1;
    msgs[0].flags = I2C_MSG_WRITE;
    msgs[1].buf = buf;
    msgs[1].len = len;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;
    return i2c_transfer(dev, msgs, 2, addr);
}

// Сканирование шины
static void scan_i2c_bus(void) {
    printk("Scanning I2C bus...\n");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy;
        struct i2c_msg msg = {.buf = &dummy, .len = 1, .flags = I2C_MSG_WRITE | I2C_MSG_STOP};
        if (i2c_transfer(i2c_dev, &msg, 1, addr) == 0) {
            printk(" Found device at: 0x%02X\n", addr);
            found++;
        }
    }
    printk(found ? " Found %d device(s)\n" : " No devices found!\n", found);
}

// Проверка и конфигурация IMU
static int check_imu(void) {
    uint8_t whoami;
    int ret;
    uint8_t addresses[2] = {0x6B, 0x6A};

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return -1;
    }

    k_msleep(100);

    for (int i = 0; i < 2; i++) {
        uint8_t addr = addresses[i];
        printk("Checking IMU at address 0x%02X...\n", addr);
        ret = i2c_read_reg(i2c_dev, addr, LSM6DS3_WHO_AM_I, &whoami);
        if (ret < 0) continue;

        printk("IMU WHO_AM_I: 0x%02X\n", whoami);
        if (whoami == 0x69 || whoami == 0x6A || whoami == 0x6B) {
            ret = i2c_write_reg(i2c_dev, addr, LSM6DS3_CTRL1_XL, 0x6A);  // ±4g, 416 Hz
            if (ret < 0) return ret;
            ret = i2c_write_reg(i2c_dev, addr, LSM6DS3_CTRL2_G, 0x6A);   // 500 dps, 416 Hz
            if (ret < 0) return ret;
            printk("IMU configured successfully at 0x%02X\n", addr);
            return 0;
        }
    }
    printk("Unknown IMU\n");
    return -1;
}

// Калибровка IMU (вызывается при старте или по BLE команде)
static void calibrate_imu(int num_samples) {
    float accel_sum[3] = {0.0f};
    float gyro_sum[3] = {0.0f};

    printk("Calibrating IMU... Keep device still for ~2 seconds.\n");

    for (int i = 0; i < num_samples; i++) {
        uint8_t buf[12];
        if (i2c_read_multi(i2c_dev, LSM6DS3_ADDR, LSM6DS3_OUTX_L_XL, buf, 12) == 0) {
            int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
            int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
            int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
            int16_t gx = (int16_t)((buf[7] << 8) | buf[6]);
            int16_t gy = (int16_t)((buf[9] << 8) | buf[8]);
            int16_t gz = (int16_t)((buf[11] << 8) | buf[10]);

            accel_sum[0] += ax;
            accel_sum[1] += ay;
            accel_sum[2] += az;
            gyro_sum[0] += gx;
            gyro_sum[1] += gy;
            gyro_sum[2] += gz;
        }
        k_msleep(10);
    }

    accel_bias[0] = accel_sum[0] / num_samples;
    accel_bias[1] = accel_sum[1] / num_samples;
    accel_bias[2] = (accel_sum[2] / num_samples) - 8192.0f;  // 1g ≈ 8192 LSB при ±4g

    gyro_bias[0] = gyro_sum[0] / num_samples;
    gyro_bias[1] = gyro_sum[1] / num_samples;
    gyro_bias[2] = gyro_sum[2] / num_samples;

    printk("Calibration done:\n");
    printk("  Accel bias: %.1f, %.1f, %.1f\n", (double)accel_bias[0], (double)accel_bias[1], (double)accel_bias[2]);
    printk("  Gyro bias:  %.1f, %.1f, %.1f\n", (double)gyro_bias[0], (double)gyro_bias[1], (double)gyro_bias[2]);
}

// Чтение данных с IMU с применением калибровки
static int read_imu_data(struct sensor_data *data) {
    uint8_t buf[12];
    int ret = i2c_read_multi(i2c_dev, LSM6DS3_ADDR, LSM6DS3_OUTX_L_XL, buf, 12);
    if (ret < 0) return ret;

    float ax = (int16_t)((buf[1] << 8) | buf[0]) - accel_bias[0];
    float ay = (int16_t)((buf[3] << 8) | buf[2]) - accel_bias[1];
    float az = (int16_t)((buf[5] << 8) | buf[4]) - accel_bias[2];
    data->accel_x = (int16_t)(ax * accel_scale[0]);
    data->accel_y = (int16_t)(ay * accel_scale[1]);
    data->accel_z = (int16_t)(az * accel_scale[2]);

    float gx = (int16_t)((buf[7] << 8) | buf[6]) - gyro_bias[0];
    float gy = (int16_t)((buf[9] << 8) | buf[8]) - gyro_bias[1];
    float gz = (int16_t)((buf[11] << 8) | buf[10]) - gyro_bias[2];
    data->gyro_x = (int16_t)gx;
    data->gyro_y = (int16_t)gy;
    data->gyro_z = (int16_t)gz;

    return 0;
}

// Вспомогательные функции для Mahony
static void normalize_quat(float *q_out) {
    float norm = sqrtf(q_out[0]*q_out[0] + q_out[1]*q_out[1] + q_out[2]*q_out[2] + q_out[3]*q_out[3]);
    if (norm > 0.0f) {
        q_out[0] /= norm;
        q_out[1] /= norm;
        q_out[2] /= norm;
        q_out[3] /= norm;
    }
}

static void quat_multiply(const float *q1, const float *q2, float *q_out) {
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void vector_cross(const float *a, const float *b, float *out) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static void vector_add(const float *a, const float *b, float *out) {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

static void vector_scale(const float *v, float s, float *out) {
    out[0] = v[0] * s;
    out[1] = v[1] * s;
    out[2] = v[2] * s;
}

static float vector_norm(const float *v) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// Mahony update
static void mahony_update(float ax, float ay, float az, float gx, float gy, float gz) {
    float accel[3] = {ax, ay, az};
    float gyro[3] = {gx, gy, gz};

    float accel_norm = vector_norm(accel);
    if (accel_norm == 0.0f) return;
    float va_meas[3];
    vector_scale(accel, 1.0f / accel_norm, va_meas);

    float q_inv[4] = {q[0], -q[1], -q[2], -q[3]};
    float gravity_world[4] = {0.0f, 0.0f, 0.0f, -1.0f};
    float temp[4];
    quat_multiply(q, gravity_world, temp);
    float va_est_quat[4];
    quat_multiply(temp, q_inv, va_est_quat);
    float va_est[3] = {va_est_quat[1], va_est_quat[2], va_est_quat[3]};

    float e[3];
    vector_cross(va_est, va_meas, e);

    float b_dot[3];
    vector_scale(e, -kI, b_dot);
    float temp_b[3];
    vector_scale(b_dot, dt, temp_b);
    float b_new[3];
    vector_add(b, temp_b, b_new);

    float omega_corr[3];
    vector_scale(e, kP, omega_corr);
    float omega_bias[3];
    vector_add(gyro, b_new, omega_bias);
    float omega_final[3];
    vector_add(omega_bias, omega_corr, omega_final);

    float p_omega[4] = {0.0f, omega_final[0], omega_final[1], omega_final[2]};
    float q_dot[4];
    quat_multiply(q, p_omega, q_dot);
    vector_scale(q_dot, 0.5f, q_dot);

    vector_scale(q_dot, dt, temp);
    vector_add(q, temp, q);
    normalize_quat(q);

    b[0] = b_new[0];
    b[1] = b_new[1];
    b[2] = b_new[2];
}

// BLE callbacks (остаются без изменений)
static void connected(struct bt_conn *conn, uint8_t err) {
    if (!err) {
        ble_connected = true;
        printk("Connected\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    ble_connected = false;
    printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static ssize_t read_sensor_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_data, sizeof(current_data));
}

static ssize_t write_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (len > 0) {
        uint8_t cmd = ((uint8_t*)buf)[0];
        if (cmd == 0x01) led_state = 1;
        else if (cmd == 0x00) led_state = 0;
        else if (cmd == 0x02 && len >= 2) data_interval_ms = ((uint8_t*)buf)[1] * 10;
        else if (cmd == 0x03) calibrate_imu(200);  // Команда калибровки по BLE
    }
    return len;
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    printk("Notifications %s\n", (value == BT_GATT_CCC_NOTIFY) ? "ENABLED" : "disabled");
}

BT_GATT_SERVICE_DEFINE(magball_service,
    BT_GATT_PRIMARY_SERVICE(&magball_service_uuid),
    BT_GATT_CHARACTERISTIC(&magball_data_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, read_sensor_data, NULL, &current_data),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&magball_cmd_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_cmd, NULL),
);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, "MagBall", 7),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MAGBALL_VAL),
};

int main(void) {
    uint32_t last_send = 0;
    uint8_t battery_level = 100;
    uint32_t counter = 0;
    uint32_t imu_error_count = 0;

    printk("\n\n========================================\n");
    printk("Magnetic Ball with IMU + Mahony + Calibration\n");
    printk("Board: promicro_nrf52840\n");
    printk("========================================\n\n");

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        printk("I2C not ready — simulated mode\n");
        current_data.imu_status = 0;
    } else {
        printk("I2C ready\n");
        scan_i2c_bus();
        if (check_imu() == 0) {
            current_data.imu_status = 1;
            calibrate_imu(200);  // Автокалибровка при старте
        } else {
            current_data.imu_status = 0;
        }
    }

    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed: %d\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");

    struct bt_le_adv_param adv_param = { .options = BT_LE_ADV_OPT_CONN, .interval_min = BT_GAP_ADV_FAST_INT_MIN_2, .interval_max = BT_GAP_ADV_FAST_INT_MAX_2 };
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed: %d\n", err);
        return 0;
    }
    printk("Advertising started\n");

    while (1) {
        if (current_data.imu_status == 1) {
            if (read_imu_data(&current_data) == 0) {
                // Перевод в физические единицы
                float accel_f[3] = {
                    current_data.accel_x * 0.000122f,  // ±4g → m/s² (примерно)
                    current_data.accel_y * 0.000122f,
                    current_data.accel_z * 0.000122f
                };
                float gyro_f[3] = {
                    current_data.gyro_x * 0.0175f * (M_PI / 180.0f),  // 500 dps → rad/s
                    current_data.gyro_y * 0.0175f * (M_PI / 180.0f),
                    current_data.gyro_z * 0.0175f * (M_PI / 180.0f)
                };

                mahony_update(accel_f[0], accel_f[1], accel_f[2], gyro_f[0], gyro_f[1], gyro_f[2]);

                current_data.quaternion[0] = q[0];
                current_data.quaternion[1] = q[1];
                current_data.quaternion[2] = q[2];
                current_data.quaternion[3] = q[3];
            } else {
                imu_error_count++;
                if (imu_error_count > 10) current_data.imu_status = 0;
            }
        } else {
            // Simulated data
            current_data.accel_x = (int16_t)(500 * sinf(counter * 0.1f));
            current_data.accel_y = (int16_t)(500 * cosf(counter * 0.1f));
            current_data.accel_z = 980 + (int16_t)(100 * sinf(counter * 0.05f));
            current_data.gyro_x = (int16_t)(100 * sinf(counter * 0.2f));
            current_data.gyro_y = (int16_t)(100 * cosf(counter * 0.2f));
            current_data.gyro_z = 0;
            current_data.quaternion[0] = 1.0f;
            current_data.quaternion[1] = current_data.quaternion[2] = current_data.quaternion[3] = 0.0f;
        }

        current_data.timestamp = k_uptime_get_32();
        current_data.sequence = sample_count++;
        current_data.battery = battery_level;

        uint32_t now = k_uptime_get_32();
        if (ble_connected && (now - last_send >= data_interval_ms)) {
            bt_gatt_notify(NULL, &magball_service.attrs[2], &current_data, sizeof(current_data));
            if (sample_count % 20 == 0) {
                printk("[%u] Acc: %d,%d,%d | Gyro: %d,%d,%d | Q: %.3f,%.3f,%.3f,%.3f\n",
                       current_data.sequence,
                       current_data.accel_x, current_data.accel_y, current_data.accel_z,
                       current_data.gyro_x, current_data.gyro_y, current_data.gyro_z,
                       (double)q[0], (double)q[1], (double)q[2], (double)q[3]);
            }
            last_send = now;
            if (sample_count % 100 == 0 && battery_level > 10) battery_level--;
        }

        counter++;
        k_msleep(10);
    }
    return 0;
}