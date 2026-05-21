#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* UUIDs MagBall */
#define BT_UUID_MAGBALL_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_MAGBALL_DATA_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdeF1)
#define BT_UUID_MAGBALL_CMD_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

static const struct bt_uuid_128 magball_service_uuid =
    BT_UUID_INIT_128(BT_UUID_MAGBALL_VAL);
static const struct bt_uuid_128 magball_data_uuid =
    BT_UUID_INIT_128(BT_UUID_MAGBALL_DATA_VAL);
static const struct bt_uuid_128 magball_cmd_uuid =
    BT_UUID_INIT_128(BT_UUID_MAGBALL_CMD_VAL);
/* Timestamp when notifications were enabled — used to skip the first 100 ms
   warm-up period so the BLE stack has time to negotiate MTU before we flood it. */
static uint32_t notify_enabled_time = 0;

/* Пакет данных шара */
#pragma pack(push, 1)
struct sensor_data {
    int16_t accel_x;   // mg или 1e-3 g
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;    // mdps или 1e-3 deg/s
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t roll;      // 1e-3 rad или deg, на STM32 можно переинтерпретировать
    int16_t pitch;
    int16_t yaw;

    uint8_t  battery;  // 0–100 %
    uint8_t  status;   // биты: 0 – IMU OK, 1 – зарядка, 2 – перегрев и т.п.
    uint16_t sequence; // счётчик пакетов
};
#pragma pack(pop)

BUILD_ASSERT(sizeof(struct sensor_data) == 22, "sensor_data must be 22 bytes");

/* Глобальные данные */
static struct sensor_data current_data;
static struct bt_conn *cur_conn = NULL;
static bool ble_connected = false;
static bool notify_enabled = false;
static uint8_t battery_level   = 100;
/* 10 ms = 100 Hz — must match dt=0.01f hard-coded in mahony_update() */
static uint32_t data_interval_ms = 10;
static uint32_t sample_count;

/* IMU / Mahony */
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float accel_bias[3] = {0};
static float gyro_bias[3]  = {0};

/* I2C / LSM6DS3 */
static const struct device *i2c_dev;
/* Actual I2C address found during check_imu() — can be 0x6A or 0x6B */
static uint8_t lsm6ds3_found_addr = LSM6DS3_ADDR;
#define LSM6DS3_ADDR      0x6B
#define LSM6DS3_WHO_AM_I  0x0F
#define LSM6DS3_CTRL1_XL  0x10
#define LSM6DS3_CTRL2_G   0x11
#define LSM6DS3_OUTX_L_XL 0x28

#define Kp 2.0f   // Доверие акселерометру
#define Ki 0.005f // Компенсация дрейфа гироскопа
static float eInt[3] = {0.0f, 0.0f, 0.0f};

/* Прототипы */
static void calibrate_imu(int num_samples);
static int  read_imu_data(struct sensor_data *data);
static void mahony_update(float ax, float ay, float az,
                          float gx, float gy, float gz);
static void quat_to_euler_zyx(const float q[4],
                              float *roll, float *pitch, float *yaw);                          

/* -------- BLE callbacks -------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        return;
    }

    cur_conn = bt_conn_ref(conn);
    ble_connected = true;
    printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ble_connected = false;
    notify_enabled = false;

    if (cur_conn) {
        bt_conn_unref(cur_conn);
        cur_conn = NULL;
    }

    printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};



static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);

    if (notify_enabled) {
        notify_enabled_time = k_uptime_get_32();
        printk("Notifications ENABLED\n");
    } else {
        printk("Notifications disabled\n");
    }
}

static ssize_t read_sensor_data(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &current_data, sizeof(current_data));
}

static ssize_t write_cmd(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    if (len > 0) {
        const uint8_t *b = buf;
        uint8_t cmd = b[0];

        if (cmd == 0x02 && len >= 2) {
            data_interval_ms = b[1] * 10;       /* шаг 10 мс */
        } else if (cmd == 0x03) {
            calibrate_imu(200);                 /* калибровка по команде */
        }
    }
    return len;
}

/* GATT‑сервис MagBall */
BT_GATT_SERVICE_DEFINE(magball_service,
    BT_GATT_PRIMARY_SERVICE(&magball_service_uuid),
    BT_GATT_CHARACTERISTIC(&magball_data_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_sensor_data, NULL, &current_data),
    BT_GATT_CCC(ccc_changed,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&magball_cmd_uuid.uuid,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL, write_cmd, NULL),
);

/* Advertising данные */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, "MagBall", 7),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MAGBALL_VAL),
};

/* -------- I2C / IMU helpers -------- */

static int i2c_write_reg(const struct device *dev,
                         uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    struct i2c_msg msg = {
        .buf   = buf,
        .len   = 2,
        .flags = I2C_MSG_WRITE | I2C_MSG_STOP,
    };
    return i2c_transfer(dev, &msg, 1, addr);
}

static int i2c_read_reg(const struct device *dev,
                        uint8_t addr, uint8_t reg, uint8_t *value)
{
    struct i2c_msg msgs[2];

    msgs[0].buf   = &reg;
    msgs[0].len   = 1;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf   = value;
    msgs[1].len   = 1;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    return i2c_transfer(dev, msgs, 2, addr);
}

static int i2c_read_multi(const struct device *dev,
                          uint8_t addr, uint8_t reg,
                          uint8_t *buf, uint16_t len)
{
    struct i2c_msg msgs[2];

    msgs[0].buf   = &reg;
    msgs[0].len   = 1;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf   = buf;
    msgs[1].len   = len;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    return i2c_transfer(dev, msgs, 2, addr);
}

static int check_imu(void)
{
    printk("check_imu() called\n");

    uint8_t who;
    uint8_t addrs[2] = {0x6B, 0x6A};

    if (!device_is_ready(i2c_dev)) {
        printk("I2C not ready\n");
        return -1;
    }

    k_msleep(50);

    for (int i = 0; i < 2; i++) {
        uint8_t a = addrs[i];
        int ret = i2c_read_reg(i2c_dev, a, LSM6DS3_WHO_AM_I, &who);
        printk("IMU WHO_AM_I @0x%02X ret=%d val=0x%02X\n", a, ret, who);

        if (ret < 0)
            continue;

        if (who == 0x69) {
            int r1 = i2c_write_reg(i2c_dev, a, LSM6DS3_CTRL1_XL, 0x6A);
            int r2 = i2c_write_reg(i2c_dev, a, LSM6DS3_CTRL2_G,  0x6A);
            printk("IMU configured at 0x%02X (writes %d,%d)\n", a, r1, r2);
            /* Save actual address for use in read_imu_data() */
            lsm6ds3_found_addr = a;
            return 0;
        }
    }

    printk("IMU not found\n");
    return -1;
}


static void calibrate_imu(int num_samples)
{
    float accel_sum[3] = {0};
    float gyro_sum[3]  = {0};
    int valid = 0;

    printk("Calibrating IMU (%d samples), keep still...\n", num_samples);

    for (int i = 0; i < num_samples; i++) {
        uint8_t buf[12];
        if (i2c_read_multi(i2c_dev, lsm6ds3_found_addr,
                           LSM6DS3_OUTX_L_XL, buf, 12) == 0) {
            int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
            int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
            int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
            int16_t gx = (int16_t)((buf[7] << 8) | buf[6]);
            int16_t gy = (int16_t)((buf[9] << 8) | buf[8]);
            int16_t gz = (int16_t)((buf[11] << 8) | buf[10]);

            accel_sum[0] += ax;
            accel_sum[1] += ay;
            accel_sum[2] += az;
            gyro_sum[0]  += gx;
            gyro_sum[1]  += gy;
            gyro_sum[2]  += gz;
            valid++;
        }
        k_msleep(10);
    }

    if (valid == 0) {
        printk("Calibration FAILED: no valid I2C reads. Biases unchanged.\n");
        return;
    }

    accel_bias[0] = accel_sum[0] / valid;
    accel_bias[1] = accel_sum[1] / valid;
    accel_bias[2] = (accel_sum[2] / valid) - 8192.0f; /* subtract 1g at ±4g scale */

    gyro_bias[0] = gyro_sum[0] / valid;
    gyro_bias[1] = gyro_sum[1] / valid;
    gyro_bias[2] = gyro_sum[2] / valid;

    printk("Calibration done (%d/%d valid). accel_bias=%.0f,%.0f,%.0f\n",
           valid, num_samples,
           (double)accel_bias[0], (double)accel_bias[1], (double)accel_bias[2]);

    /* Reset Mahony filter state to prevent startup spike */
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    eInt[0] = 0.0f; eInt[1] = 0.0f; eInt[2] = 0.0f;
}

static int read_imu_data(struct sensor_data *data)
{
    uint8_t buf[12];

    if (i2c_read_multi(i2c_dev, lsm6ds3_found_addr,
                       LSM6DS3_OUTX_L_XL, buf, 12) < 0)
        return -1;

    float ax = (int16_t)((buf[1] << 8) | buf[0]) - accel_bias[0];
    float ay = (int16_t)((buf[3] << 8) | buf[2]) - accel_bias[1];
    float az = (int16_t)((buf[5] << 8) | buf[4]) - accel_bias[2];

    float gx = (int16_t)((buf[7] << 8) | buf[6]) - gyro_bias[0];
    float gy = (int16_t)((buf[9] << 8) | buf[8]) - gyro_bias[1];
    float gz = (int16_t)((buf[11] << 8) | buf[10]) - gyro_bias[2];

    data->accel_x = (int16_t)ax;
    data->accel_y = (int16_t)ay;
    data->accel_z = (int16_t)az;
    data->gyro_x  = (int16_t)gx;
    data->gyro_y  = (int16_t)gy;
    data->gyro_z  = (int16_t)gz;

    /* Перевод в g и rad/s для фильтра */
    float ax_g = ax * 0.000122f;                      
    float ay_g = ay * 0.000122f;
    float az_g = az * 0.000122f;
    float gx_r = gx * 0.0175f * (M_PI / 180.0f);      
    float gy_r = gy * 0.0175f * (M_PI / 180.0f);
    float gz_r = gz * 0.0175f * (M_PI / 180.0f);

    /* Кормим данные в фильтр. 
       ВАЖНО: мы передаем ax_g, а не ax_n, так как фильтр сам сделает нормализацию! */
    mahony_update(ax_g, ay_g, az_g, gx_r, gy_r, gz_r);

    /* Достаем чистые, отфильтрованные углы (ZYX) */
    float roll, pitch, yaw;
    quat_to_euler_zyx(q, &roll, &pitch, &yaw);

    /* Запаковываем для отправки (в тысячных долях радиана или градуса) */
    data->roll  = (int16_t)lrintf(roll  * 1000.0f);  
    data->pitch = (int16_t)lrintf(pitch * 1000.0f);
    data->yaw   = (int16_t)lrintf(yaw   * 1000.0f);

    return 0;
}

/* Очень упрощённый Mahony: только нормализация кватерниона и интеграция гиры */
static void mahony_update(float ax, float ay, float az,
                          float gx, float gy, float gz)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float dt = 0.01f; // 10ms (100 Гц)

    // Нормализация вектора акселерометра
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    // Оценка направления гравитации по текущему кватерниону
    vx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    vy = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    // Векторное произведение (ошибка)
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    // Интегральная ошибка
    eInt[0] += ex * Ki * dt;
    eInt[1] += ey * Ki * dt;
    eInt[2] += ez * Ki * dt;

    // Коррекция гироскопа
    gx += Kp * ex + eInt[0];
    gy += Kp * ey + eInt[1];
    gz += Kp * ez + eInt[2];

    // Интеграция скорости изменения кватерниона
    float dq0 = 0.5f * (-q[1]*gx - q[2]*gy - q[3]*gz);
    float dq1 = 0.5f * ( q[0]*gx + q[2]*gz - q[3]*gy);
    float dq2 = 0.5f * ( q[0]*gy - q[1]*gz + q[3]*gx);
    float dq3 = 0.5f * ( q[0]*gz + q[1]*gy - q[2]*gx);

    q[0] += dq0 * dt; q[1] += dq1 * dt;
    q[2] += dq2 * dt; q[3] += dq3 * dt;

    // Нормализация кватерниона
    norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
}
/* q = [w, x, y, z], выдаём углы в радианах (ZYX: yaw-pitch-roll) */
static void quat_to_euler_zyx(const float q[4],
                              float *roll, float *pitch, float *yaw)
{
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    /* roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    /* pitch (y-axis rotation) */
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        *pitch = asinf(sinp);
    }

    /* yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}
/* -------- main -------- */

int main(void)
{
    printk("\n=== MagBall / promicro_nrf52840 ===\n");

    /* IMU init */
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (device_is_ready(i2c_dev) && (check_imu() == 0)) {
        current_data.status |= 0x01;
        calibrate_imu(200);
        printk("IMU OK (addr=0x%02X)\n", lsm6ds3_found_addr);
    } else {
        current_data.status &= ~0x01;
        printk("IMU not detected, using simulated data\n");
    }

    /* BLE init */
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed: %d\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");

    /* Connectable advertising через готовый пресет BT_LE_ADV_CONN */
    struct bt_le_adv_param adv_param = {
    .options = BT_LE_ADV_OPT_CONN,   /* connectable undirected adv */
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed: %d\n", err);
        return 0;
    }
    printk("Advertising as MagBall\n");

    /* Основной цикл */
    while (1) {
        /* Обновление IMU или симуляция */
        if ((current_data.status & 0x01) &&
            (read_imu_data(&current_data) == 0)) {

            /* read_imu_data уже заполнил accel/gyro и roll/pitch/yaw */

        } else {
            /* Симуляция */
            float t = sample_count * 0.1f;
            current_data.accel_x = (int16_t)(500 * sinf(t));
            current_data.accel_y = (int16_t)(500 * cosf(t));
            current_data.accel_z = 980;
            current_data.gyro_x  = 0;
            current_data.gyro_y  = 0;
            current_data.gyro_z  = 0;

            current_data.roll  = 0;
            current_data.pitch = 0;
            current_data.yaw   = 0;
        }

        current_data.sequence = sample_count++;
        current_data.battery  = battery_level;
        
        /* Send BLE notification. Skip first 100 ms after subscription to give
           the stack time to negotiate MTU before we start flooding packets. */
        if (ble_connected && notify_enabled &&
            (k_uptime_get_32() - notify_enabled_time > 100U)) {
            int ret = bt_gatt_notify(cur_conn, &magball_service.attrs[2],
                                     &current_data, sizeof(current_data));
            if (ret && ret != -EAGAIN) {
                /* -EAGAIN = TX queue full — normal at 100 Hz, skip silently */
                printk("Notify error: %d\n", ret);
            }
        }
        /* Log every 100 packets to avoid UART spam at 100 Hz */
        if (sample_count % 100 == 0) {
            printk("BALL seq=%lu roll=%d pitch=%d yaw=%d\n",
                   (unsigned long)sample_count,
                   current_data.roll,
                   current_data.pitch,
                   current_data.yaw);
        }

        k_msleep(10);
    }

    return 0;
}
