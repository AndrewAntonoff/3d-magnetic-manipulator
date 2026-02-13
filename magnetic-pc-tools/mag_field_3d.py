import serial
import serial.tools.list_ports
import numpy as np
from scipy.interpolate import RBFInterpolator
import matplotlib.pyplot as plt
from matplotlib import animation
import threading
import time
from collections import deque

# ===== НАСТРОЙКИ =====
BAUDRATE = 115200
TIMEOUT = 1
HISTORY_LEN = 50

sensor_coords = np.array([
    [40.0, 0.0, 20.0],
    [0.0, 40.0, 20.0],
    [-40.0, 0.0, 20.0],
    [0.0, -40.0, 20.0],
    [0.0, 0.0, 0.0]
])

BOWL_DIAM = 110.0
BOWL_DEPTH = 40.0
R_SPHERE = (BOWL_DIAM**2 / (8 * BOWL_DEPTH) + BOWL_DEPTH / 2)
CENTER_Z = - (R_SPHERE - BOWL_DEPTH)

X_MIN, X_MAX = -60, 60
Y_MIN, Y_MAX = -60, 60
Z_MIN, Z_MAX = -5, 45
GRID_RES = 15

data_queue = deque(maxlen=1)
running = True
history = [deque(maxlen=HISTORY_LEN) for _ in range(5)]
time_history = deque(maxlen=HISTORY_LEN)

# Интерактивный масштаб
manual_scale = None

def on_key(event):
    global manual_scale
    if event.key == '+':
        if manual_scale is None:
            manual_scale = 1.0
        else:
            manual_scale += 0.1
        print(f"Масштаб: {manual_scale:.2f}")
    elif event.key == '-':
        if manual_scale is None:
            manual_scale = 1.0
        else:
            manual_scale = max(0.1, manual_scale - 0.1)
        print(f"Масштаб: {manual_scale:.2f}")
    elif event.key == 'a':
        manual_scale = None
        print("Автоматический масштаб")

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
            print(f"Выбран порт: {port.device}")
            return port.device
    try:
        idx = int(input("Выберите номер порта: "))
        return ports[idx].device
    except:
        return None

def read_serial(ser):
    global running
    while running:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) == 1 + 3 * 5:
                try:
                    timestamp = int(parts[0])
                    values = np.array([float(x) for x in parts[1:]]).reshape(5, 3)
                    data_queue.append((timestamp, values))
                except ValueError:
                    pass
            else:
                print(f"Игнорируется: {line}")
        except Exception as e:
            print(f"Ошибка чтения: {e}")
            break

def create_spherical_bowl():
    phi = np.linspace(0, 2*np.pi, 30)
    theta = np.linspace(0, np.arccos(1 - BOWL_DEPTH / R_SPHERE), 15)
    Phi, Theta = np.meshgrid(phi, theta)
    X = R_SPHERE * np.sin(Theta) * np.cos(Phi)
    Y = R_SPHERE * np.sin(Theta) * np.sin(Phi)
    Z = -R_SPHERE * np.cos(Theta) + CENTER_Z + R_SPHERE
    return X, Y, Z

def create_grid():
    x = np.linspace(X_MIN, X_MAX, GRID_RES)
    y = np.linspace(Y_MIN, Y_MAX, GRID_RES)
    z = np.linspace(Z_MIN, Z_MAX, GRID_RES)
    Xg, Yg, Zg = np.meshgrid(x, y, z, indexing='ij')
    points = np.vstack([Xg.ravel(), Yg.ravel(), Zg.ravel()]).T
    return points, (Xg, Yg, Zg)

def interpolate_field(points, sensor_values):
    B_norm = np.linalg.norm(sensor_values, axis=1)
    rbf = RBFInterpolator(sensor_coords, B_norm, kernel='thin_plate_spline', epsilon=5)
    B_grid = rbf(points)
    return B_grid

fig = plt.figure(figsize=(14, 8))
ax3d = fig.add_subplot(2, 2, 1, projection='3d')
ax_plot = fig.add_subplot(2, 2, 2)
ax_info = fig.add_subplot(2, 2, 3)
ax_info.axis('off')
ax_extra = fig.add_subplot(2, 2, 4)
ax_extra.axis('off')

grid_points, (Xg, Yg, Zg) = create_grid()

def init_plot():
    Xb, Yb, Zb = create_spherical_bowl()
    ax3d.plot_wireframe(Xb, Yb, Zb, color='gray', alpha=0.3, linewidth=0.5, label='Чаша')
    ax3d.scatter(sensor_coords[:,0], sensor_coords[:,1], sensor_coords[:,2],
                 color='blue', s=30, label='Датчики')
    ax3d.set_xlim(X_MIN, X_MAX)
    ax3d.set_ylim(Y_MIN, Y_MAX)
    ax3d.set_zlim(Z_MIN, Z_MAX)
    ax3d.set_xlabel('X (мм)')
    ax3d.set_ylabel('Y (мм)')
    ax3d.set_zlabel('Z (мм)')
    ax3d.set_title('3D-вид')

    ax_plot.set_xlabel('Время (отсчёты)')
    ax_plot.set_ylabel('|B| (µT)')
    ax_plot.set_title('Модуль поля')
    ax_plot.grid(True)

def animate(frame):
    if len(data_queue) == 0:
        return
    timestamp, values = data_queue[-1]
    B_norm = np.linalg.norm(values, axis=1)

    time_history.append(timestamp)
    for i in range(5):
        history[i].append(B_norm[i])

    # Масштаб векторов
    if manual_scale is None:
        max_val = np.max(np.abs(values))
        scale = 10.0 / max_val if max_val > 0 else 1.0
    else:
        scale = manual_scale

    # Очистка и перерисовка 3D
    ax3d.clear()
    Xb, Yb, Zb = create_spherical_bowl()
    ax3d.plot_wireframe(Xb, Yb, Zb, color='gray', alpha=0.3, linewidth=0.5)
    ax3d.scatter(sensor_coords[:,0], sensor_coords[:,1], sensor_coords[:,2],
                 color='blue', s=30)

    # Интерполяция и сечения
    B_grid = interpolate_field(grid_points, values).reshape(GRID_RES, GRID_RES, GRID_RES)

    z_plane = 20
    iz = np.argmin(np.abs(np.linspace(Z_MIN, Z_MAX, GRID_RES) - z_plane))
    X, Y = np.meshgrid(np.linspace(X_MIN, X_MAX, GRID_RES), np.linspace(Y_MIN, Y_MAX, GRID_RES))
    B_slice = B_grid[:, :, iz].T
    ax3d.contour(X, Y, B_slice, zdir='z', offset=z_plane, levels=8, cmap='plasma', alpha=0.6)

    y_plane = 0
    iy = np.argmin(np.abs(np.linspace(Y_MIN, Y_MAX, GRID_RES) - y_plane))
    X, Z = np.meshgrid(np.linspace(X_MIN, X_MAX, GRID_RES), np.linspace(Z_MIN, Z_MAX, GRID_RES))
    B_slice = B_grid[:, iy, :].T
    ax3d.contour(X, B_slice, Z, zdir='y', offset=y_plane, levels=8, cmap='plasma', alpha=0.6)

    x_plane = 0
    ix = np.argmin(np.abs(np.linspace(X_MIN, X_MAX, GRID_RES) - x_plane))
    Y, Z = np.meshgrid(np.linspace(Y_MIN, Y_MAX, GRID_RES), np.linspace(Z_MIN, Z_MAX, GRID_RES))
    B_slice = B_grid[ix, :, :].T
    ax3d.contour(B_slice, Y, Z, zdir='x', offset=x_plane, levels=8, cmap='plasma', alpha=0.6)

    # Векторы
    for i, (x, y, z) in enumerate(sensor_coords):
        bx, by, bz = values[i] * scale
        ax3d.quiver(x, y, z, bx, by, bz, color='red', length=1.0,
                    normalize=False, linewidth=2, alpha=0.8)

    # Текстовые метки
    for i, (x, y, z) in enumerate(sensor_coords):
        txt = f'{i}: X={values[i,0]:.1f}\nY={values[i,1]:.1f}\nZ={values[i,2]:.1f}'
        ax3d.text(x, y, z + 5, txt, fontsize=7, color='black', ha='center')

    ax3d.set_xlim(X_MIN, X_MAX)
    ax3d.set_ylim(Y_MIN, Y_MAX)
    ax3d.set_zlim(Z_MIN, Z_MAX)
    ax3d.set_xlabel('X (мм)')
    ax3d.set_ylabel('Y (мм)')
    ax3d.set_zlabel('Z (мм)')
    ax3d.set_title('3D-вид')

    # Графики
    ax_plot.clear()
    ax_plot.set_xlabel('Время (отсчёты)')
    ax_plot.set_ylabel('|B| (µT)')
    ax_plot.set_title('Модуль поля')
    ax_plot.grid(True)
    t = list(range(len(time_history)))
    for i in range(5):
        ax_plot.plot(t, list(history[i]), label=f'Sensor {i}')
    ax_plot.legend(loc='upper right')

    # Информация
    ax_info.clear()
    ax_info.axis('off')
    ax_info.text(0.1, 0.9, 'Текущие значения датчиков:', transform=ax_info.transAxes,
                 fontsize=12, weight='bold')
    ypos = 0.8
    for i in range(5):
        txt = f'Sensor {i}:  X={values[i,0]:6.1f} µT  Y={values[i,1]:6.1f} µT  Z={values[i,2]:6.1f} µT  |B|={B_norm[i]:6.1f} µT'
        ax_info.text(0.1, ypos, txt, transform=ax_info.transAxes, fontsize=9, family='monospace')
        ypos -= 0.1

    # Доп. информация
    ax_extra.clear()
    ax_extra.axis('off')
    ax_extra.text(0.1, 0.5, f'Timestamp: {timestamp} ms', transform=ax_extra.transAxes, fontsize=10)
    ax_extra.text(0.1, 0.4, f'Scale: {scale:.2f} ({"auto" if manual_scale is None else "manual"})',
                  transform=ax_extra.transAxes, fontsize=10)
    ax_extra.text(0.1, 0.3, 'Клавиши: + / - / a', transform=ax_extra.transAxes, fontsize=9, color='gray')

def main():
    global running
    port = find_serial_port()
    if not port:
        return

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
        print(f"Подключено к {port}")
    except serial.SerialException as e:
        print(f"Ошибка открытия порта: {e}")
        return

    ser.write(b"start_stream\n")
    print("Команда start_stream отправлена. Ожидание данных...")
    time.sleep(1)

    thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
    thread.start()

    while len(data_queue) == 0:
        time.sleep(0.1)
    print("Получены первые данные, запуск анимации...")

    init_plot()
    fig.canvas.mpl_connect('key_press_event', on_key)
    ani = animation.FuncAnimation(fig, animate, interval=100, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    running = False
    ser.close()
    print("Программа завершена.")

if __name__ == "__main__":
    main()