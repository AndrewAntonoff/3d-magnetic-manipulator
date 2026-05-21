import serial
import struct
import time
import numpy as np
import signal
import sys

# --- Настройки ---
PORT = 'COM15'
BAUD = 921600
# СВЯЗЬ СТАБИЛЬНА! Возвращаем боевую мощность (до 7 Ампер)
MAX_OUTPUT = 0.7  
MARKER = struct.pack('<I', 0xDEADBEEF)

OFFSET_X, OFFSET_Y, OFFSET_Z = 0.0, 0.0, 0.0

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.last_derivative = 0.0 # Для фильтрации выбросов

    def compute(self, current_value):
        now = time.time()
        dt = now - self.last_time
        if dt < 0.005: dt = 0.005
            
        error = self.setpoint - current_value
        self.integral = np.clip(self.integral + error * dt, -1.0, 1.0)
        
        # Считаем сырую скорость
        raw_derivative = (error - self.last_error) / dt
        
        # === Low-Pass Filter (Фильтр низких частот) для D-компоненты ===
        # Сглаживает "пинки" ПИДа (берем 20% нового значения и 80% старого)
        derivative = (0.2 * raw_derivative) + (0.8 * self.last_derivative)
        
        self.last_error = error
        self.last_time = now
        self.last_derivative = derivative
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

def estimate_position(sensors):
    x_raw = sensors[3] - sensors[9]   
    y_raw = sensors[7] - sensors[13]
    z_raw = sensors[2]               
    return (x_raw - OFFSET_X), (y_raw - OFFSET_Y), (z_raw - OFFSET_Z)

# Глобальная переменная для амортизатора тока
prev_powers = np.zeros(12, dtype=np.float32)

def compute_coil_powers(ux, uy, uz, current_limit):
    global prev_powers
    
    target_powers = np.zeros(12, dtype=np.float32)
    target_powers[0], target_powers[2] = ux, -ux
    target_powers[1], target_powers[3] = uy, -uy
    
    # БАЗОВЫЙ ТОК ПОДЪЕМА (Feed-Forward). Подбирай его так, 
    # чтобы при Err_Z = 0 шар был почти невесомым.
    uz_bias = 0.30 
    uz_final = uz + uz_bias
    
    for i in range(4, 12): 
        target_powers[i] = uz_final
        
    target_powers = np.clip(target_powers, -current_limit, current_limit)
    
    # Slew Rate Limiter (Амортизатор)
    max_delta = 0.02 
    delta = np.clip(target_powers - prev_powers, -max_delta, max_delta)
    
    final_powers = prev_powers + delta
    prev_powers = final_powers 
    
    return final_powers

def calibrate_home_position(ser):
    global OFFSET_X, OFFSET_Y, OFFSET_Z
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
            
        ser.write(MARKER + struct.pack('<12f', *[0.0]*12))
        
        s = struct.unpack('<15f', raw)
        samples.append([(s[3]-s[9]), (s[7]-s[13]), s[2]])
        timeouts = 0
        
        if len(samples) % 30 == 0:
            print(f"Прогресс: {int(len(samples)/150*100)}%")
            
    avg = np.mean(samples, axis=0)
    OFFSET_X, OFFSET_Y, OFFSET_Z = avg[0], avg[1], avg[2]
    print(f"\nГОТОВО! База зафиксирована: Z={OFFSET_Z:.1f}")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"Ошибка порта: {e}")
        return

    ser.write(b"\r\ntrain_off\r\n")
    time.sleep(0.5)
    ser.reset_input_buffer()

    print("\n" + "="*50)
    print("ЭТАП 1: КАЛИБРОВКА ТОЧКИ ДОМА")
    print("Поместите шар 25мм в центр каркаса и держите его рукой.")
    print("Калибровка начнется через 3 секунды...")
    print("="*50)
    time.sleep(3)

    ser.reset_input_buffer()
    print("Отправка команды train_on...")
    for char in b"train_on\n":
        ser.write(bytes([char]))
        time.sleep(0.005)
    time.sleep(0.1) 
    ser.reset_input_buffer() 
    
    calibrate_home_position(ser)

    # Включаем "накопление" (Ki = 0.005) для всех осей!
    # Kp сделал чуть меньше, чтобы регулятор не бесился от больших цифр
    # Делаем стенки очень мягкими и вязкими
    pid_x = PID(0.01, 0.001, 0.01)
    pid_y = PID(0.01, 0.001, 0.01)
    pid_z = PID(0.15, 0.01, 0.05) 

    def stop_handler(sig, frame):
        ser.write(MARKER + struct.pack('<12f', *[0.0]*12))
        time.sleep(0.05)
        ser.write(b"\r\ntrain_off\r\n")
        ser.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, stop_handler)

    print("Теперь плавно отпускайте шар. Запуск ПИД...")
    
    timeouts = 0
    current_limit = 0.0 
    
    try:
        while True:
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
            
            if current_limit < MAX_OUTPUT:
                current_limit += 0.005 
            
            sensors = struct.unpack('<15f', raw)
            dx, dy, dz = estimate_position(sensors)
            
            # --- НОРМАЛИЗАЦИЯ МАСШТАБА ---
            # Делим гигантские тысячи на 200, чтобы получить адекватные цифры (10-40)
            dx = dx / 200.0
            dy = dy / 200.0
            
            ux = pid_x.compute(dx)
            uy = pid_y.compute(dy)
            uz = pid_z.compute(dz)
            
            powers = compute_coil_powers(ux, uy, uz, current_limit)
            ser.write(MARKER + struct.pack('<12f', *powers))

            if time.time() % 1 < 0.05:
                # ВЫВОДИМ РЕНТГЕН ВСЕХ ОСЕЙ!
                # Err_X/Y - это куда сместился шар. Coil_X - это как реагирует катушка 0
                print(f"E_X: {dx:6.1f} | E_Y: {dy:6.1f} | E_Z: {dz:6.1f} || C_X: {powers[0]:5.2f} | C_Z: {powers[4]:5.2f}")
                
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        ser.write(b"\r\ntrain_off\r\n")
        ser.close()

if __name__ == "__main__":
    main()