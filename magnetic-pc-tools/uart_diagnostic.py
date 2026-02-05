import serial
import time
import sys

def test_uart_communication(port='COM16'):
    """Прямой тест UART соединения"""
    print(f"=== ПРЯМОЙ ТЕСТ UART СОЕДИНЕНИЯ ===")
    print(f"Порт: {port}")
    print("="*40)
    
    try:
        # Открываем соединение
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=0.5,
            write_timeout=2,
            rtscts=False,      # Отключаем RTS/CTS
            dsrdtr=False,      # Отключаем DSR/DTR
            xonxoff=False      # Отключаем программный контроль потока
        )
        
        # Ждем инициализации
        time.sleep(2)
        
        print("1. Чтение начальных данных...")
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"   Получено: {len(data)} байт")
            try:
                print(data.decode('utf-8'))
            except:
                print("   (бинарные данные)")
        
        print("\n2. Тест отправки символов...")
        
        # Тест 1: Отправляем по одному символу
        print("   Отправка 'h'...")
        ser.write(b'h')
        time.sleep(0.1)
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"   Ответ: {data}")
        
        print("   Отправка 'e'...")
        ser.write(b'e')
        time.sleep(0.1)
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"   Ответ: {data}")
        
        print("   Отправка 'l'...")
        ser.write(b'l')
        time.sleep(0.1)
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"   Ответ: {data}")
        
        print("   Отправка 'p'...")
        ser.write(b'p')
        time.sleep(0.1)
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"   Ответ: {data}")
        
        print("\n3. Тест отправки команды по частям...")
        print("   Отправка 'help'...")
        ser.write(b'help')
        time.sleep(0.5)
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"   Ответ: {data[:100]}")
        
        print("\n4. Тест с разными окончаниями строк...")
        
        endings = [
            ('\\r\\n', b'\r\n'),
            ('\\n', b'\n'),
            ('\\r', b'\r'),
        ]
        
        for name, ending in endings:
            print(f"   Отправка 'help' + {name}")
            ser.write(b'help' + ending)
            time.sleep(0.5)
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                print(f"   Ответ: {data[:100]}")
        
        print("\n5. Тест с включением/выключением DTR/RTS...")
        
        # Пробуем разные комбинации DTR/RTS
        dtr_rts_combinations = [
            (False, False),
            (True, False),
            (False, True),
            (True, True),
        ]
        
        for dtr, rts in dtr_rts_combinations:
            print(f"   DTR={dtr}, RTS={rts}")
            ser.dtr = dtr
            ser.rts = rts
            time.sleep(0.1)
            
            ser.write(b'\n')
            time.sleep(0.1)
            
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                print(f"   Ответ: {data[:50]}")
        
        ser.close()
        
    except Exception as e:
        print(f"Ошибка: {e}")

def hardware_reset_test(port='COM16'):
    """Тест сброса оборудования через DTR"""
    print(f"\n=== ТЕСТ СБРОСА ОБОРУДОВАНИЯ ===")
    print(f"Порт: {port}")
    print("="*40)
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=2
        )
        
        # Включаем DTR (может вызвать сброс на некоторых платах)
        print("1. Включаем DTR (может вызвать сброс)...")
        ser.dtr = True
        time.sleep(0.5)
        ser.dtr = False
        time.sleep(2)  # Ждем перезагрузки
        
        print("2. Читаем вывод после сброса...")
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            try:
                print(data.decode('utf-8'))
            except:
                print(f"Бинарные данные: {data[:200]}")
        
        ser.close()
        
    except Exception as e:
        print(f"Ошибка: {e}")

def send_raw_data_test(port='COM16'):
    """Отправка сырых данных для тестирования"""
    print(f"\n=== ТЕСТ СЫРЫХ ДАННЫХ ===")
    print(f"Порт: {port}")
    print("="*40)
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=1
        )
        
        # Даем время на инициализацию
        time.sleep(2)
        
        # Очищаем буфер
        if ser.in_waiting:
            ser.read(ser.in_waiting)
        
        test_patterns = [
            ("Пустая строка", b''),
            ("Один символ 'h'", b'h'),
            ("Символ новой строки", b'\n'),
            ("Символ возврата каретки", b'\r'),
            ("CR+LF", b'\r\n'),
            ("Множественные символы", b'\n\n\n'),
            ("help с CR", b'help\r'),
            ("help с LF", b'help\n'),
            ("help с CR+LF", b'help\r\n'),
            ("status с CR+LF", b'status\r\n'),
        ]
        
        for desc, data in test_patterns:
            print(f"\nТест: {desc}")
            print(f"   Отправка: {data}")
            
            if data:
                ser.write(data)
            
            # Ждем ответа
            time.sleep(0.5)
            
            # Читаем ответ
            response = b''
            start_time = time.time()
            while time.time() - start_time < 1:
                if ser.in_waiting:
                    chunk = ser.read(ser.in_waiting)
                    response += chunk
                    time.sleep(0.01)
            
            if response:
                try:
                    decoded = response.decode('utf-8', errors='ignore')
                    print(f"   Ответ: {decoded[:100]}")
                except:
                    print(f"   Ответ (бинарный): {response[:100]}")
            else:
                print("   Нет ответа")
        
        ser.close()
        
    except Exception as e:
        print(f"Ошибка: {e}")

def main():
    print("="*60)
    print("ДИАГНОСТИКА UART СОЕДИНЕНИЯ С STM32")
    print("="*60)
    
    port = "COM16"
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"\nИспользуется порт: {port}")
    
    while True:
        print("\n" + "="*60)
        print("МЕНЮ ДИАГНОСТИКИ:")
        print("1. Прямой тест UART соединения")
        print("2. Тест сброса оборудования (DTR)")
        print("3. Тест отправки сырых данных")
        print("4. Проверка всех доступных портов")
        print("5. Выход")
        print("="*60)
        
        choice = input("\nВыберите опцию (1-5): ").strip()
        
        if choice == '1':
            test_uart_communication(port)
            
        elif choice == '2':
            hardware_reset_test(port)
            
        elif choice == '3':
            send_raw_data_test(port)
            
        elif choice == '4':
            check_all_ports()
            
        elif choice == '5':
            print("Выход...")
            break
            
        else:
            print("Неверный выбор")

def check_all_ports():
    """Проверка всех COM портов"""
    import winreg
    
    print("\n=== ПРОВЕРКА ВСЕХ COM ПОРТОВ ===")
    
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
        ports = []
        i = 0
        while True:
            try:
                val = winreg.EnumValue(key, i)
                ports.append(val[1])
                i += 1
            except WindowsError:
                break
        
        if not ports:
            print("COM порты не найдены")
            return
        
        print(f"Найдено портов: {len(ports)}")
        
        for port in ports:
            print(f"\nПроверка порта {port}...")
            try:
                # Быстрая проверка
                ser = serial.Serial(port, 115200, timeout=1)
                time.sleep(0.5)
                
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    print(f"   Есть данные: {len(data)} байт")
                    if len(data) > 0:
                        try:
                            decoded = data.decode('utf-8', errors='ignore')
                            if "MAGNETIC" in decoded or "STM32" in decoded:
                                print(f"   ✅ Найден STM32 Magnetic Manipulator!")
                                print(f"   {decoded[:100]}...")
                            else:
                                print(f"   Данные: {decoded[:50]}")
                        except:
                            print(f"   Бинарные данные")
                else:
                    print("   Нет данных")
                
                ser.close()
                
            except Exception as e:
                print(f"   Ошибка: {e}")
    
    except Exception as e:
        print(f"Ошибка при проверке портов: {e}")

if __name__ == "__main__":
    main()