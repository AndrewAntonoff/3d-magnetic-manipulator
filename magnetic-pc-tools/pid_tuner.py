import os
import json
import serial
import serial.tools.list_ports
import struct
import time
import numpy as np
import signal
import sys

# --- НАСТРОЙКИ ПО УМОЛЧАНИЮ ---
DEFAULT_CONFIG_FILE = "pid_config.json"
BAUD = 921600
MAX_OUTPUT = 0.7
MARKER = struct.pack('<I', 0xDEADBEEF)

# Файл конфигурации, который можно редактировать "на лету" без перезапуска скрипта
default_config = {
    "pid_x": {"kp": 0.3, "ki": 0.0, "kd": 0.05, "setpoint": 0.0},
    "pid_y": {"kp": 0.3, "ki": 0.0, "kd": 0.05, "setpoint": 0.0},
    "pid_z": {"kp": 0.5, "ki": 0.01, "kd": 0.05, "setpoint": 0.0},
    "uz_bias": 0.0,
    "invert_xy_polarity": True, # True как в neuro_train2.py, False как в neuro_PID.py
    "normalize_xy": False # Делить ли dx/dy на 200 (True для neuro_PID.py, False для neuro_train2.py)
}

def ensure_config():
    if not os.path.exists(DEFAULT_CONFIG_FILE):
        with open(DEFAULT_CONFIG_FILE, 'w') as f:
            json.dump(default_config, f, indent=4)
        print(f"[{DEFAULT_CONFIG_FILE}] Создан файл конфигурации по умолчанию.")
        return default_config
    else:
        try:
            with open(DEFAULT_CONFIG_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"[ОШИБКА ЧТЕНИЯ КОНФИГА]: {e}")
            return default_config

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.last_derivative = 0.0

    def load_config(self, config_dict):
        self.kp = config_dict.get("kp", self.kp)
        self.ki = config_dict.get("ki", self.ki)
        self.kd = config_dict.get("kd", self.kd)
        self.setpoint = config_dict.get("setpoint", self.setpoint)

    def compute(self, current_value):
        now = time.time()
        dt = now - self.last_time
        if dt < 0.005: dt = 0.005
            
        error = self.setpoint - current_value
        self.integral = np.clip(self.integral + error * dt, -1.0, 1.0)
        
        raw_derivative = (error - self.last_error) / dt
        derivative = (0.2 * raw_derivative) + (0.8 * self.last_derivative)
        
        self.last_error = error
        self.last_time = now
        self.last_derivative = derivative
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class LevitationController:
    def __init__(self):
        self.config = ensure_config()
        self.last_config_mod_time = os.path.getmtime(DEFAULT_CONFIG_FILE) if os.path.exists(DEFAULT_CONFIG_FILE) else 0
        
        # Инициализация ПИДов из конфига
        self.pid_x = PID(**self.config["pid_x"])
        self.pid_y = PID(**self.config["pid_y"])
        self.pid_z = PID(**self.config["pid_z"])
        
        self.offset_x, self.offset_y, self.offset_z = 0.0, 0.0, 0.0
        self.prev_powers = np.zeros(12, dtype=np.float32)

    def reload_config_if_changed(self):
        try:
            if not os.path.exists(DEFAULT_CONFIG_FILE): return
            mtime = os.path.getmtime(DEFAULT_CONFIG_FILE)
            if mtime > self.last_config_mod_time:
                with open(DEFAULT_CONFIG_FILE, 'r') as f:
                    new_config = json.load(f)
                self.config = new_config
                self.pid_x.load_config(self.config["pid_x"])
                self.pid_y.load_config(self.config["pid_y"])
                self.pid_z.load_config(self.config["pid_z"])
                self.last_config_mod_time = mtime
                print("\n[CONFIG RELOADED] ПИД-коэффициенты обновлены на лету!")
        except Exception as e:
            pass # Игнорируем ошибки (например файл открыт редактором)

    def estimate_position(self, sensors):
        x_raw = sensors[3] - sensors[9]   
        y_raw = sensors[7] - sensors[13]
        z_raw = sensors[2]               
        dx = x_raw - self.offset_x
        dy = y_raw - self.offset_y
        dz = z_raw - self.offset_z
        
        if self.config.get("normalize_xy", False):
            dx /= 200.0
            dy /= 200.0
            
        return dx, dy, dz

    def compute_coil_powers(self, ux, uy, uz, current_limit):
        target_powers = np.zeros(12, dtype=np.float32)
        
        if self.config.get("invert_xy_polarity", True):
            target_powers[0], target_powers[2] = -ux, ux
            target_powers[1], target_powers[3] = -uy, uy
        else:
            target_powers[0], target_powers[2] = ux, -ux
            target_powers[1], target_powers[3] = uy, -uy
        
        uz_final = uz + self.config.get("uz_bias", 0.0)
        
        for i in range(4, 12): 
            target_powers[i] = uz_final
            
        target_powers = np.clip(target_powers, -current_limit, current_limit)
        
        # Slew Rate Limiter (Амортизатор)
        max_delta = 0.02 
        delta = np.clip(target_powers - self.prev_powers, -max_delta, max_delta)
        final_powers = self.prev_powers + delta
        self.prev_powers = final_powers 
        
        return final_powers

def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    for port in ports:
        if "CH340" in port.description or "USB Serial" in port.description:
            print(f"Найден порт: {port.device}")
            return port.device
    print("Список портов:")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device}")
    try:
        idx = int(input("Введите номер порта: "))
        return ports[idx].device
    except:
        return None

def calibrate_home_position(ser, controller):
    samples = []
    print("Замер данных (не шевелите шар)...")
    timeouts = 0
    
    while len(samples) < 150:
        junk = ser.read_until(MARKER)
        if not junk.endswith(MARKER):
            print("X", end="", flush=True)
            timeouts += 1
            if timeouts > 15:
                print("\n[ОШИБКА] Превышен лимит ошибок калибровки.")
                sys.exit(1)
            continue
            
        raw = ser.read(60)
        if len(raw) != 60: continue
            
        # Поддержание связи нулевыми токами
        ser.write(MARKER + struct.pack('<12f', *[0.0]*12))
        
        s = struct.unpack('<15f', raw)
        samples.append([(s[3]-s[9]), (s[7]-s[13]), s[2]])
        timeouts = 0
        
        if len(samples) % 30 == 0:
            print(f"Осталось: {150 - len(samples)}")
            
    avg = np.mean(samples, axis=0)
    controller.offset_x, controller.offset_y, controller.offset_z = avg[0], avg[1], avg[2]
    print(f"\nГОТОВО! База: X={controller.offset_x:.1f}, Y={controller.offset_y:.1f}, Z={controller.offset_z:.1f}")

def main():
    port = find_serial_port()
    if not port:
        print("Подключение отменено (нет портов).")
        return

    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except Exception as e:
        print(f"Ошибка порта: {e}")
        return
        
    controller = LevitationController()

    ser.write(b"\r\ntrain_off\r\n")
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("\n" + "="*50)
    print("АВТО ТЮНИНГ ПИД (LIVE RELOAD)")
    print(f"1. Редактируйте файл '{DEFAULT_CONFIG_FILE}'")
    print("2. При сохранении файла, ПИД обновится мгновенно!")
    print("ЭТАП 1: КАЛИБРОВКА. Поместите шар в центр каркаса.")
    print("Калибровка начнется через 3 секунды...")
    print("="*50)
    time.sleep(3)

    ser.reset_input_buffer()
    for char in b"train_on\n":
        ser.write(bytes([char]))
        time.sleep(0.005)
    time.sleep(0.1) 
    ser.reset_input_buffer() 
    
    calibrate_home_position(ser, controller)

    def stop_handler(sig, frame):
        print("\nЗавершение...")
        try:
            ser.write(MARKER + struct.pack('<12f', *[0.0]*12))
            time.sleep(0.05)
            ser.write(b"\r\ntrain_off\r\n")
            ser.close()
        except:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, stop_handler)

    print("\nТеперь плавно отпускайте шар. Запуск системы управления...")
    
    timeouts = 0
    current_limit = 0.0 
    last_reload_time = time.time()
    
    try:
        while True:
            # Проверка файла JSON раз в 1 секунду
            if time.time() - last_reload_time > 1.0:
                controller.reload_config_if_changed()
                last_reload_time = time.time()
            
            junk = ser.read_until(MARKER)
            if not junk.endswith(MARKER): 
                timeouts += 1
                if timeouts > 10:
                    print("\n[АВАРИЯ] Контроллер разорвал связь (таймаут).")
                    break
                continue
                
            raw = ser.read(60)
            if len(raw) != 60: continue
            timeouts = 0 
            
            # Плавный запуск тока (soft-start)
            if current_limit < MAX_OUTPUT:
                current_limit += 0.005 
            
            sensors = struct.unpack('<15f', raw)
            dx, dy, dz = controller.estimate_position(sensors)
            
            ux = controller.pid_x.compute(dx)
            uy = controller.pid_y.compute(dy)
            uz = controller.pid_z.compute(dz)
            
            powers = controller.compute_coil_powers(ux, uy, uz, current_limit)
            ser.write(MARKER + struct.pack('<12f', *powers))

            if time.time() % 1 < 0.05:
                # Показываем данные в строку без лишних переходов
                print(f"\rZ: Err={dz:6.1f} | Coil_Z={powers[4]:5.2f} | X: Err={dx:6.1f} | Y: Err={dy:6.1f} | lim={current_limit:.2f}", end="", flush=True)
                
    except Exception as e:
        print(f"\nОшибка цикла: {e}")
    finally:
        stop_handler(None, None)

if __name__ == "__main__":
    main()
