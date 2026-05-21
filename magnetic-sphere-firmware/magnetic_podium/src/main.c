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

static const struct device *uart_stm32;   // UART1 для передачи данных на STM32
static const uint8_t packet_header[2] = {0xAA, 0x55};
static struct bt_gatt_exchange_params mtu_exchange_params;

/* Delayed work for non-blocking reconnect */
static struct k_work_delayable reconnect_work;


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



/* --- Вспомогательные функции --- */

/* Функция для надежной отправки данных в UART */
static void uart_send_data(const uint8_t *data, size_t len)
{
    if (!uart_stm32) return;
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_stm32, data[i]);
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

        /* 1. Отправляем маркер начала пакета */
        uart_send_data(packet_header, 2);
        
        /* 2. Отправляем бинарные данные в аппаратный UART0 */
        uart_send_data((const uint8_t *)&ball_data, sizeof(ball_data));

        /* Логи в USB CDC (printk) */
        if (data_count % 20 == 0) {
            printk("Received %u packets, seq=%u\n", data_count, ball_data.sequence);
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

    /* ШАГ 1: Ищем саму характеристику DATA */
    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *chrc = attr->user_data;

        if (!bt_uuid_cmp(chrc->uuid, &magball_data_uuid.uuid)) {
            printk("Found MagBall DATA characteristic, handle=0x%04x\n", 
                   chrc->value_handle);

            // Сохраняем handle характеристики
            memset(&subscribe_params, 0, sizeof(subscribe_params));
            subscribe_params.value_handle = chrc->value_handle;

            /* Меняем параметры поиска, чтобы теперь найти дескриптор CCCD */
            discover_params.uuid = NULL; 
            discover_params.start_handle = chrc->value_handle + 1;
            discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

            int err = bt_gatt_discover(conn, &discover_params);
            if (err) {
                printk("Discover CCCD failed (err %d)\n", err);
            }
            return BT_GATT_ITER_STOP; // Останавливаем поиск характеристик
        }
    }

    /* ШАГ 2: Ищем дескриптор CCCD для найденной характеристики */
    if (params->type == BT_GATT_DISCOVER_DESCRIPTOR) {
        if (!bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC)) {
            printk("Found CCCD descriptor, handle=0x%04x\n", attr->handle);
            
            // Теперь у нас есть оба handle, подписываемся по-настоящему
            subscribe_params.ccc_handle = attr->handle;
            subscribe_params.value = BT_GATT_CCC_NOTIFY;
            subscribe_params.notify = notify_cb;

            int err = bt_gatt_subscribe(conn, &subscribe_params);
            if (err) {
                printk("Subscribe failed (err %d)\n", err);
            } else {
                current_state = STATE_CONNECTED;
                data_count = 0;
                printk("Subscribed to notifications SUCCESS!\n");

                /* Включаем турбо-скорость BLE только после успешной подписки */
                struct bt_le_conn_param fast_param = {
                    .interval_min = 6,  /* 7.5 ms */
                    .interval_max = 6,  /* 7.5 ms */
                    .latency = 0,
                    .timeout = 400,
                };
                bt_conn_le_param_update(conn, &fast_param);
            }
            return BT_GATT_ITER_STOP; // Останавливаем поиск дескрипторов
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

    /* Check both name AND service UUID to avoid false connections */
    static const struct bt_uuid_128 target_svc_uuid =
        BT_UUID_INIT_128(BT_UUID_MAGBALL_SERVICE_VAL);

    bool name_match = (name[0] && strstr(name, "MagBall"));
    bool uuid_match = false;

    /* Re-parse AD to look for the service UUID */
    net_buf_simple_restore(ad, &state);
    while (ad->len > 1) {
        uint8_t len     = net_buf_simple_pull_u8(ad);
        if (len == 0 || len > ad->len) break;
        uint8_t ad_type = net_buf_simple_pull_u8(ad);
        len--;

        if ((ad_type == BT_DATA_UUID128_ALL || ad_type == BT_DATA_UUID128_SOME)
            && len == 16) {
            if (memcmp(ad->data, target_svc_uuid.val, 16) == 0) {
                uuid_match = true;
            }
        }
        net_buf_simple_pull(ad, len);
    }
    net_buf_simple_restore(ad, &state);

    if (name_match && uuid_match) {
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

// Добавить ПЕРЕД функцией connected(...):
static void exchange_func(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_exchange_params *params)
{
    printk("MTU exchange %s (err %u)\n", err == 0 ? "successful" : "failed", err);

    /* ТОЛЬКО ПОСЛЕ расширения MTU начинаем искать характеристики! */
    memset(&discover_params, 0, sizeof(discover_params));
    discover_params.uuid         = &magball_data_uuid.uuid;
    discover_params.func         = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    int disc_err = bt_gatt_discover(conn, &discover_params);
    if (disc_err) {
        printk("bt_gatt_discover failed (err %d)\n", disc_err);
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

    /* ЗАПРАШИВАЕМ РАСШИРЕНИЕ MTU */
    /* Сами характеристики будем искать уже внутри exchange_func */
    mtu_exchange_params.func = exchange_func;
    int mtu_err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
    if (mtu_err) {
        printk("MTU exchange request failed (err %d)\n", mtu_err);
    }
}

static void reconnect_work_handler(struct k_work *work)
{
    /* This runs in the system workqueue context — safe to call start_scan */
    start_scan();
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

    /* Exponential backoff — non-blocking via workqueue instead of k_msleep */
    uint32_t delay_ms = 1000U * (1U << (reconnect_attempts - 1));
    printk("Reconnecting in %u ms\n", delay_ms);
    k_work_schedule(&reconnect_work, K_MSEC(delay_ms));
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
    
    /* Initialize non-blocking reconnect work */
    k_work_init_delayable(&reconnect_work, reconnect_work_handler);

    uart_stm32 = DEVICE_DT_GET(DT_NODELABEL(uart1));
    if (!device_is_ready(uart_stm32)) {
        printk("UART1 not ready, check hardware\n");
    } else {
        printk("UART1 ready for STM32 communication\n");
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