#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

static const struct device *uart_dev;

/* UUID сервиса и характеристики MagBall (как на шаре) */
#define BT_UUID_MAGBALL_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_MAGBALL_DATA_VAL \
    BT_UUID_128_ENCODE(0xf0bda123, 0x1234, 0x5678, 0x1234, 0x56789abcdeF1)

static const struct bt_uuid_128 magball_data_uuid =
    BT_UUID_INIT_128(BT_UUID_MAGBALL_DATA_VAL);

/* Структура данных — как на шаре */
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

/* Состояние системы */
enum system_state {
    STATE_SCANNING,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_READY,
    STATE_LEVITATING,
    STATE_ERROR
};

static enum system_state current_state = STATE_SCANNING;

/* Глобальные переменные */
static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static struct sensor_data ball_data;
static bool new_data_available;
static uint32_t data_count;

static uint32_t reconnect_attempts;
static const uint32_t MAX_RECONNECT_ATTEMPTS = 5;

/* Флаг для управления отправкой данных в UART */
static bool enable_uart_output = true;

/* --- Вспомогательные функции --- */

/* Функция для надежной отправки данных в UART */
static void uart_send_data(const uint8_t *data, size_t len)
{
    if (!uart_dev || !enable_uart_output) {
        return;
    }

    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

/* --- Notification callback --- */
static uint8_t notify_cb(struct bt_conn *conn,
                         struct bt_gatt_subscribe_params *params,
                         const void *data, uint16_t length)
{
    if (!data) {
        printk("Notifications disabled\n");
        return BT_GATT_ITER_STOP;
    }

    if (length == sizeof(struct sensor_data)) {
        memcpy(&ball_data, data, sizeof(ball_data));
        new_data_available = true;
        data_count++;

        /* Отправляем бинарные данные в UART */
        uart_send_data((const uint8_t *)&ball_data, sizeof(ball_data));

        /* Выводим отладочную информацию */
        if (data_count % 20 == 0) {
            printk("Received %u packets, seq=%u, bat=%u%%\n", 
                   data_count, ball_data.sequence, ball_data.battery);
        }

        if (current_state == STATE_CONNECTED && data_count >= 30) {
            current_state = STATE_READY;
            printk("System READY for levitation\n");
        }
    } else {
        printk("Unexpected len=%u, expected %u\n",
               length, (unsigned int)sizeof(struct sensor_data));
    }

    return BT_GATT_ITER_CONTINUE;
}

/* --- Discover callback --- */

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("Discover complete\n");
        memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *chrc = attr->user_data;

        if (!bt_uuid_cmp(chrc->uuid, &magball_data_uuid.uuid)) {
            printk("Found MagBall DATA characteristic, handle=0x%04x\n", 
                   chrc->value_handle);

            /* Подписываемся на уведомления */
            memset(&subscribe_params, 0, sizeof(subscribe_params));
            subscribe_params.ccc_handle = 0; /* Будем искать CCCD */
            subscribe_params.value_handle = chrc->value_handle;
            subscribe_params.value = BT_GATT_CCC_NOTIFY;
            subscribe_params.notify = notify_cb;
            subscribe_params.subscribe = bt_gatt_subscribe;

            int err = bt_gatt_subscribe(conn, &subscribe_params);
            if (err) {
                printk("Subscribe failed (err %d)\n", err);
            } else {
                current_state = STATE_CONNECTED;
                data_count = 0;
                printk("Subscribed to notifications\n");
            }

            memset(params, 0, sizeof(*params));
            return BT_GATT_ITER_STOP;
        }
    }

    /* Если это дескриптор CCCD */
    if (params->type == BT_GATT_DISCOVER_DESCRIPTOR) {
        const struct bt_gatt_attr *desc = attr;
        
        if (!bt_uuid_cmp(desc->uuid, BT_UUID_GATT_CCC)) {
            printk("Found CCCD, handle=0x%04x\n", attr->handle);
            subscribe_params.ccc_handle = attr->handle;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

/* --- Сканирование --- */

static void start_scan(void);

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    if (type == BT_GAP_ADV_TYPE_ADV_NONCONN_IND ||
        type == BT_GAP_ADV_TYPE_SCAN_RSP) {
        return;
    }

    struct net_buf_simple_state state;
    net_buf_simple_save(ad, &state);

    char name[32] = {0};

    while (ad->len > 1) {
        uint8_t len = net_buf_simple_pull_u8(ad);
        if (len == 0) {
            break;
        }
        if (len > ad->len) {
            break;
        }

        uint8_t ad_type = net_buf_simple_pull_u8(ad);
        len--;

        if (ad_type == BT_DATA_NAME_COMPLETE ||
            ad_type == BT_DATA_NAME_SHORTENED) {
            uint8_t n = MIN(len, sizeof(name) - 1);
            memcpy(name, ad->data, n);
            name[n] = '\0';
        }

        net_buf_simple_pull(ad, len);
    }

    net_buf_simple_restore(ad, &state);

    if (name[0] && strstr(name, "MagBall")) {
        printk("Found MagBall: %s RSSI=%d, connecting...\n", addr_str, rssi);

        bt_le_scan_stop();

        struct bt_le_conn_param *param =
            BT_LE_CONN_PARAM(6, 12, 0, 400);

        current_state = STATE_CONNECTING;

        int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                    param, &default_conn);
        if (err) {
            printk("bt_conn_le_create failed (err %d)\n", err);
            current_state = STATE_SCANNING;
            start_scan();
        }
    }
}

static void start_scan(void)
{
    struct bt_le_scan_param scan_param = {
        .type     = BT_LE_SCAN_TYPE_ACTIVE,
        .options  = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_WINDOW,
    };

    int err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) {
        printk("Starting scan failed (err %d)\n", err);
    } else {
        printk("Scanning for MagBall...\n");
        current_state = STATE_SCANNING;
    }
}

/* --- Callbacks подключения --- */

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (err) {
        printk("Failed to connect to %s (err %u)\n", addr_str, err);
        current_state = STATE_SCANNING;
        start_scan();
        return;
    }

    printk("Connected to %s\n", addr_str);
    reconnect_attempts = 0;

    if (!default_conn) {
        default_conn = bt_conn_ref(conn);
    }

    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid         = &magball_data_uuid.uuid;
    discover_params.func         = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    err = bt_gatt_discover(default_conn, &discover_params);
    if (err) {
        printk("bt_gatt_discover failed (err %d)\n", err);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    printk("Disconnected from %s (reason 0x%02x)\n", addr_str, reason);

    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    current_state = STATE_SCANNING;

    reconnect_attempts++;
    if (reconnect_attempts > MAX_RECONNECT_ATTEMPTS) {
        printk("Max reconnect attempts reached\n");
        current_state = STATE_ERROR;
        return;
    }

    uint32_t delay_ms = 1000U * (1U << (reconnect_attempts - 1));
    printk("Reconnecting in %u ms\n", delay_ms);
    k_msleep(delay_ms);
    start_scan();
}

BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* --- Поток управления --- */

static void control_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    printk("Control task started\n");

    while (1) {
        if (new_data_available) {
            new_data_available = false;

            /* Здесь можно добавить реальную обработку для левитации */
            if (current_state == STATE_READY && data_count >= 50) {
                current_state = STATE_LEVITATING;
                printk("Starting levitation control!\n");
            }
        }

        k_msleep(10);
    }
}

/* --- Основная функция --- */

#define CONTROL_STACK_SIZE 4096
#define CONTROL_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
static struct k_thread control_thread;

int main(void)
{
    int err;

    printk("\n\n========================================\n");
    printk("   Magnetic Ball Podium Controller\n");
    printk("   Board: promicro_nrf52840\n");
    printk("========================================\n\n");

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");

    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart_dev)) {
        printk("Console UART not ready\n");
        uart_dev = NULL;
    } else {
        printk("Console UART ready for binary data at 115200 baud\n");
    }

    start_scan();

    k_thread_create(&control_thread, control_stack,
                    K_THREAD_STACK_SIZEOF(control_stack),
                    control_task,
                    NULL, NULL, NULL,
                    CONTROL_THREAD_PRIORITY, 0, K_NO_WAIT);

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}