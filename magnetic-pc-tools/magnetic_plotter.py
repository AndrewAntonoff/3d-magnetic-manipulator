import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import csv
from datetime import datetime

class MagneticFieldPlotter:
    def __init__(self, port='COM10', baudrate=115200, max_points=500):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.max_points = max_points
        
        # Инициализация данных
        self.timestamps = deque(maxlen=max_points)
        self.x_data = deque(maxlen=max_points)
        self.y_data = deque(maxlen=max_points)
        self.z_data = deque(maxlen=max_points)
        self.magnitude = deque(maxlen=max_points)
        
        # Статистика
        self.start_time = time.time()
        self.sample_count = 0
        
        # Настройка графика
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Цвета
        self.colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']
        
        # Файл для сохранения данных
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f'magnetic_data_{timestamp}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'time_s', 'x_ut', 'y_ut', 'z_ut', 'magnitude'])
        
        print("Magnetic Field Plotter initialized")
        print(f"Saving data to: magnetic_data_{timestamp}.csv")
        
    def parse_data(self, line):
        """Парсинг строки данных"""
        try:
            parts = line.strip().split(',')
            if len(parts) == 3:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                mag = np.sqrt(x**2 + y**2 + z**2)
                return x, y, z, mag
        except (ValueError, IndexError):
            pass
        return None
    
    def update_plot(self, frame):
        """Обновление графика"""
        # Чтение всех доступных данных
        while self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8', errors='ignore')
            data = self.parse_data(line)
            
            if data:
                x, y, z, mag = data
                current_time = time.time() - self.start_time
                
                # Сохранение данных
                self.timestamps.append(current_time)
                self.x_data.append(x)
                self.y_data.append(y)
                self.z_data.append(z)
                self.magnitude.append(mag)
                
                # Запись в CSV
                self.csv_writer.writerow([
                    datetime.now().isoformat(),
                    f"{current_time:.3f}",
                    f"{x:.2f}",
                    f"{y:.2f}",
                    f"{z:.2f}",
                    f"{mag:.2f}"
                ])
                
                self.sample_count += 1
                
                # Обновление заголовка
                self.fig.suptitle(
                    f'Magnetic Field Monitor | Samples: {self.sample_count} | '
                    f'Rate: {self.sample_count/current_time:.1f} Hz | '
                    f'Current: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Mag={mag:.1f} µT',
                    fontsize=12
                )
        
        # Очистка графиков
        self.ax1.clear()
        self.ax2.clear()
        
        if len(self.timestamps) > 1:
            # График компонентов
            self.ax1.plot(self.timestamps, self.x_data, 
                        label=f'X (avg: {np.mean(self.x_data):.1f} µT)',
                        color=self.colors[0], linewidth=1.5)
            self.ax1.plot(self.timestamps, self.y_data,
                        label=f'Y (avg: {np.mean(self.y_data):.1f} µT)',
                        color=self.colors[1], linewidth=1.5)
            self.ax1.plot(self.timestamps, self.z_data,
                        label=f'Z (avg: {np.mean(self.z_data):.1f} µT)',
                        color=self.colors[2], linewidth=1.5)
            
            self.ax1.set_xlabel('Time (s)')
            self.ax1.set_ylabel('Magnetic Field (µT)')
            self.ax1.set_title('Magnetic Field Components')
            self.ax1.legend(loc='upper right')
            self.ax1.grid(True, alpha=0.3)
            
            # График магнитуды
            self.ax2.plot(self.timestamps, self.magnitude,
                        label=f'Magnitude (avg: {np.mean(self.magnitude):.1f} µT)',
                        color=self.colors[3], linewidth=2)
            
            # Отображение максимального значения
            max_mag = max(self.magnitude) if self.magnitude else 0
            if max_mag > 50:  # Если есть значительное поле
                self.ax2.axhline(y=max_mag, color='red', linestyle='--', alpha=0.5,
                                label=f'Max: {max_mag:.1f} µT')
            
            self.ax2.set_xlabel('Time (s)')
            self.ax2.set_ylabel('Magnitude (µT)')
            self.ax2.set_title('Total Magnetic Field Strength')
            self.ax2.legend(loc='upper right')
            self.ax2.grid(True, alpha=0.3)
            
            # Автомасштабирование
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax2.relim()
            self.ax2.autoscale_view()
        
        plt.tight_layout()
    
    def start(self):
        """Запуск анимации"""
        print("Starting plot... Close window to stop.")
        
        # Отправка команды начала потока
        self.serial_port.write(b'start_stream 50\n')
        time.sleep(0.5)
        
        # Очистка буфера
        self.serial_port.reset_input_buffer()
        
        # Запуск анимации
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, cache_frame_data=False)
        plt.show()
    
    def stop(self):
        """Остановка и очистка"""
        self.serial_port.write(b'stop_stream\n')
        time.sleep(0.5)
        self.csv_file.close()
        self.serial_port.close()
        print("Plotter stopped. Data saved.")

# Запуск скрипта
if __name__ == "__main__":
    import sys
    
    # Настройка порта
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = 'COM3'  # Измените на ваш порт
    
    plotter = MagneticFieldPlotter(port=port, baudrate=115200)
    
    try:
        plotter.start()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        plotter.stop()