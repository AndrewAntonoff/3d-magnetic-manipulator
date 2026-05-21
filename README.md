# 3D Magnetic Manipulator

Система магнитной левитации для управления шаром в 3D-пространстве.

## Архитектура

```
MagBall (nRF52840)  ──── BLE ────  Подиум (nRF52840)  ──── UART ────  STM32H750
  IMU LSM6DS3                        BLE-Central                         8× MLX90393
  Mahony AHRS                        Relay → STM32                       8× катушка
  BLE Peripheral                                                          VL53L5X (TOF)
  100 Гц нотификации                                                      QSPI Flash
                                                                          USB HID
```

## Компоненты репозитория

| Папка | Описание |
|-------|---------|
| `magnetic-podium-firmware/` | Прошивка STM32H750VBT6 (CubeIDE) — управление катушками, опрос датчиков, PID-левитация |
| `magnetic-sphere-firmware/magnetic_ball/` | Прошивка шара nRF52840 (Zephyr) — IMU + BLE Peripheral |
| `magnetic-sphere-firmware/magnetic_podium/` | Прошивка подиума nRF52840 (Zephyr) — BLE Central → UART relay |
| `magnetic-pc-tools/` | Python-инструменты: калибровка катушек, RL-обучение |
| `3d-magnetic-manipulator/` | CAD-модели корпуса и механики |

## Аппаратная часть (STM32)

| Компонент | Кол-во | Интерфейс |
|-----------|--------|-----------|
| STM32H750VBT6 | 1 | — |
| MLX90393 (магнитный датчик) | 8 | SPI1, CS: PC0–PC7 |
| Катушки (H-мост MOSFET) | 8 | TIM1/8 PWM + GPIO DIR |
| VL53L5CX (TOF дальномер) | 1 | I2C1 |
| nRF52840 подиум (BLE-реле) | 1 | UART2 (PA2/PA3, 115200) |
| W25Q256 QSPI Flash | 1 | QUADSPI |
| USB Full Speed | 1 | HID — SpaceMouse / Joystick |

## Прошивка STM32 — возможности

- **DMA-опрос** 8 датчиков MLX90393 с медианным + EMA фильтром
- **PID-контроллер** с раздельными контурами позиции и ориентации
- **Оценка позиции** шара по взвешенному вектору магнитного поля
- **Матрица калибровки катушек** (`flash_coil`) с линейной интерполяцией по току
- **Консольный интерфейс UART** (help, sensor, coil, levitate, calibrate, stream…)
- **USB HID** — SpaceMouse и Joystick (`usb_mode 0/1`)
- **Бинарный протокол** обмена с Python (маркер `0xDEADBEEF`)
- **Training mode** — режим сбора данных для RL-обучения

## Прошивка шара (Zephyr / nRF52840)

- IMU **LSM6DS3** по I2C (автоопределение адреса 0x6A / 0x6B)
- **Mahony AHRS** — кватернионный фильтр, 100 Гц
- BLE GATT Peripheral: нотификации 22-байт пакета @ 7.5 мс
- Автокалибровка смещений акселерометра/гироскопа при старте (200 семплов)

## Прошивка подиума (Zephyr / nRF52840)

- BLE Central: активное сканирование с фильтром по имени **и** UUID сервиса
- Relay: BLE-нотификация → UART (заголовок `0xAA 0x55` + 22 байта IMU)
- Экспоненциальный backoff при переподключении через `k_work_schedule` (неблокирующий)

## Консольные команды (UART 115200)

```
help                        — список команд
status                      — состояние системы
sensor <n>                  — показания датчика n (0–7)
coil <n> <power>            — ток катушки n (0.0–1.0)
coil_all <power>            — все катушки сразу
coil_off                    — выключить все катушки
levitate                    — запустить PID-левитацию
stop_levitate               — остановить левитацию
calibrate [n]               — калибровка смещений датчиков
save_cal / load_cal         — сохранить/загрузить калибровку в Flash
start_stream [ms]           — потоковые данные (интервал 10–1000 мс)
stop_stream                 — остановить поток
flash_coil                  — загрузить матрицу калибровки катушек
set_target <x> <y> <z>      — целевая позиция шара
set_pid_pos <ax> <Kp> <Ki> <Kd>
train_on / train_off        — режим обучения RL
usb_mode <0|1>              — 0=Joystick, 1=SpaceMouse
```

## Сборка

**STM32 (STM32CubeIDE):**
```
File → Import → Existing Projects → magnetic-podium-firmware/
Project → Build All  (Ctrl+B)
```

**Zephyr (шар / подиум):**
```bash
west build -b promicro_nrf52840 magnetic-sphere-firmware/magnetic_ball
west build -b promicro_nrf52840 magnetic-sphere-firmware/magnetic_podium
```

## TODO

- [ ] Откалибровать `B_permanent` для датчиков 5–7 (команды `sensor 5`, `sensor 6`, `sensor 7` без шара и катушек)
- [ ] Реализовать `set_pid_ori` для управления ориентацией шара
- [ ] Заменить блокирующий UART в `SystemControlLoop` на DMA/прерывания

