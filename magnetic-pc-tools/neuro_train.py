#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import struct
import time
import torch
import torch.nn as nn
import numpy as np
import csv
import signal
import sys

BAUD = 921600  # Updated to match the reliable 921600 from PID scripts
LOG_FILE = "levitation_data.csv"
SAVE_INTERVAL = 100
NUM_SENSORS = 15
NUM_COILS = 12
MARKER = struct.pack('<I', 0xDEADBEEF)

class LevitationNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(NUM_SENSORS, 64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.ReLU(),
            nn.Linear(128, NUM_COILS),
            nn.Tanh()
        )
    def forward(self, x):
        return self.net(x)

def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("Не найдено ни одного COM-порта!")
        return None
    print("Доступные порты:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    for port in ports:
        if "CH340" in port.description or "USB Serial" in port.description:
            print(f"Выбран порт по умолчанию: {port.device}")
            return port.device
    
    # If not automatically found, let the user pick
    if len(ports) == 1:
        return ports[0].device
        
    try:
        idx = int(input("Выберите номер порта: "))
        return ports[idx].device
    except:
        return None

def sync_levitation_fast(ser, model):
    """
    Ожидает маркер 0xDEADBEEF от контроллера, затем читает 60 байт датчиков,
    вычисляет токи катушек нейросетью и отправляет их обратно с маркером.
    Возвращает (sensors, coil_powers) или None при ошибке/таймауте.
    """
    # Ждем маркер
    junk = ser.read_until(MARKER)
    if not junk.endswith(MARKER):
        return None
        
    # Читаем ровно 60 байт (15 floats x 4)
    raw_sensors = ser.read(60)
    if len(raw_sensors) != 60:
        return None

    sensors = struct.unpack('<15f', raw_sensors)

    with torch.no_grad():
        action = model(torch.tensor(sensors, dtype=torch.float32))
    coil_powers = action.numpy().astype(np.float32)
    
    # Отправляем ответ с маркером (4 + 48 = 52 байта)
    # ИСПРАВЛЕН БАГ: добавлен MARKER и убрана двойная отправка
    ser.write(MARKER + struct.pack('<12f', *coil_powers))

    return sensors, coil_powers
    
def init_csv(filename):
    import os
    if not os.path.isfile(filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            header = [f'S{i//3}{"xyz"[i%3]}' for i in range(15)] + [f'C{i}' for i in range(12)]
            writer.writerow(header)
        print(f"Создан файл {filename}")

def main():
    print("Инициализация нейросети (случайные веса)...")
    model = LevitationNet()
    
    port = find_serial_port()
    if not port:
        return
        
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        print(f"Подключено к {port} на скорости {BAUD}")
    except Exception as e:
        print(f"Ошибка подключения: {e}")
        return

    # ВКЛЮЧАЕМ РЕЖИМ ОБУЧЕНИЯ НА КОНТРОЛЛЕРЕ
    ser.write(b"\r\ntrain_off\r\n")
    time.sleep(0.5)
    ser.reset_input_buffer()
    
    # Отправка команды
    ser.reset_input_buffer()
    print("Отправка команды train_on...")
    for char in b"train_on\n":
        ser.write(bytes([char]))
        time.sleep(0.005)
    time.sleep(0.1) 
    ser.reset_input_buffer()

    init_csv(LOG_FILE)

    data_buffer = []

    running = True

    def signal_handler(sig, frame):
        nonlocal running
        if running:
            print("\nПрерывание по Ctrl+C. Завершаем безопасно...")
            running = False

    signal.signal(signal.SIGINT, signal_handler)

    print("\nПрограмма запущена. Нажмите Ctrl+C для выхода.")
    iteration = 0
    timeouts = 0
    
    try:
        while running:
            result = sync_levitation_fast(ser, model)
            if result:
                timeouts = 0
                sensors, coils = result
                data_buffer.append(list(sensors) + list(coils))
                iteration += 1
                
                if iteration % SAVE_INTERVAL == 0:
                    with open(LOG_FILE, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerows(data_buffer)
                    data_buffer.clear()
                    
                if time.time() % 1.0 < 0.05:
                    print(f"\rИтераций: {iteration} | Coils: " + " ".join([f"{v:+.2f}" for v in coils[:4]]), end="", flush=True)
            else:
                timeouts += 1
                if timeouts > 50:
                    print("\n[АВАРИЯ] Потеряна связь с контроллером!")
                    break
    finally:
        # Сохраняем остаток данных
        if data_buffer:
            with open(LOG_FILE, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(data_buffer)

        # ВЫКЛЮЧАЕМ РЕЖИМ ОБУЧЕНИЯ
        print("\nОстановка...")
        ser.write(MARKER + struct.pack('<12f', *[0.0]*12))
        time.sleep(0.05)
        ser.write(b"\r\ntrain_off\r\n")
        time.sleep(0.1)
        ser.close()
        print("Скрипт завершён.")

if __name__ == "__main__":
    main()