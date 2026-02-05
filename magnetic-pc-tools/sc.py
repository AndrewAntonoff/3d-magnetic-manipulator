import serial
import time
import sys

class SimpleMagneticManipulator:
    def __init__(self, port='COM16', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        
    def connect(self):
        """Подключение к устройству"""
        try:
            print(f"Подключение к {self.port}...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=2,
                write_timeout=2
            )
            
            # Ждем инициализации
            time.sleep(2)
            
            # Очищаем буфер
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                print(f"Получено при подключении: {data[:200]}")
            
            print("✅ Подключено!")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка подключения: {e}")
            return False
    
    def send_command_raw(self, command, wait_time=1.0):
        """Отправка команды с разными вариантами окончаний"""
        if not self.ser:
            return None
        
        # Пробуем разные окончания строк
        endings = [
            '\r\n',    # Стандартное окончание
            '\n',      # Только новая строка
            '\r',      # Только возврат каретки
            '',        # Без окончания
        ]
        
        responses = []
        
        for ending in endings:
            full_command = command + ending
            print(f"\nПробуем команду: '{command}' с окончанием: {repr(ending)}")
            
            try:
                # Очищаем входной буфер
                if self.ser.in_waiting:
                    self.ser.read(self.ser.in_waiting)
                
                # Отправляем команду
                bytes_sent = self.ser.write(full_command.encode())
                print(f"Отправлено байт: {bytes_sent}")
                
                # Ждем ответа
                time.sleep(wait_time)
                
                # Читаем ответ
                response_bytes = b''
                start_time = time.time()
                
                while time.time() - start_time < 2:  # Максимум 2 секунды
                    if self.ser.in_waiting:
                        chunk = self.ser.read(self.ser.in_waiting)
                        response_bytes += chunk
                        time.sleep(0.01)
                    else:
                        time.sleep(0.01)
                
                if response_bytes:
                    try:
                        response_text = response_bytes.decode('utf-8', errors='ignore')
                        responses.append((ending, response_text))
                        print(f"Получен ответ ({len(response_text)} символов):")
                        print(response_text[:200])
                        if len(response_text) > 200:
                            print("...")
                    except:
                        print(f"Получены бинарные данные: {response_bytes[:50]}")
                else:
                    print("Нет ответа")
                    
            except Exception as e:
                print(f"Ошибка при отправке: {e}")
        
        return responses
    
    def interactive_mode(self):
        """Интерактивный режим"""
        print("\n" + "="*60)
        print("ИНТЕРАКТИВНЫЙ РЕЖИМ УПРАВЛЕНИЯ")
        print("="*60)
        print("Доступные команды:")
        print("  help, status, hwstatus, testsensors, testcoils, testspi")
        print("  coil 1 0.1, stopall, monitor 100, stopmonitor")
        print("  testall, calibrate, emergency, reset")
        print("\nВведите 'exit' для выхода")
        print("="*60)
        
        command_history = []
        
        while True:
            try:
                # Читаем команду
                cmd = input("\n> ").strip()
                
                if cmd.lower() == 'exit':
                    break
                elif not cmd:
                    continue
                
                # Добавляем в историю
                command_history.append(cmd)
                
                # Отправляем команду
                print(f"\nОтправка команды: '{cmd}'")
                
                # Очищаем буфер
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    print(f"Очищено буфера: {len(data)} байт")
                
                # Отправляем команду с \r\n окончанием (стандартное для терминалов)
                full_cmd = cmd + '\r\n'
                self.ser.write(full_cmd.encode())
                
                # Ждем ответа с таймаутом
                response = b''
                start_time = time.time()
                
                # Увеличиваем таймаут для длинных ответов
                timeout = 3.0 if 'testsensors' in cmd.lower() else 2.0
                
                while time.time() - start_time < timeout:
                    if self.ser.in_waiting:
                        chunk = self.ser.read(self.ser.in_waiting)
                        response += chunk
                        # Если получили данные, ждем еще немного
                        if chunk:
                            time.sleep(0.05)
                    else:
                        # Если данных нет и прошло время, выходим
                        if response and (time.time() - start_time > 0.5):
                            break
                        time.sleep(0.01)
                
                if response:
                    try:
                        decoded = response.decode('utf-8', errors='ignore')
                        print(f"\nОтвет ({len(decoded)} символов):")
                        print(decoded)
                    except:
                        print(f"\nБинарный ответ ({len(response)} байт):")
                        print(response.hex())
                else:
                    print("⚠️ Нет ответа")
                
                # Небольшая пауза между командами
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("\n\nПрервано пользователем")
                break
            except Exception as e:
                print(f"\nОшибка: {e}")
    
    def quick_test(self):
        """Быстрый тест всех команд"""
        test_commands = [
            ("Проверка связи", "", 1),
            ("Команда help", "help", 1),
            ("Статус системы", "status", 1),
            ("Статус оборудования", "hwstatus", 2),
            ("Тест датчиков", "testsensors", 3),
            ("Тест катушек", "testcoils", 2),
            ("Тест SPI", "testspi", 2),
            ("Остановка катушек", "stopall", 1),
        ]
        
        print("\n" + "="*60)
        print("БЫСТРЫЙ ТЕСТ КОМАНД")
        print("="*60)
        
        for test_name, command, wait_time in test_commands:
            print(f"\n{test_name}: '{command}'")
            
            if command:
                self.send_command_raw(command, wait_time)
            else:
                # Просто ждем и читаем, что есть в буфере
                time.sleep(1)
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    try:
                        print(data.decode('utf-8', errors='ignore')[:200])
                    except:
                        print(f"Данные: {data[:100]}")
    
    def close(self):
        """Закрытие соединения"""
        if self.ser and self.ser.is_open:
            # Пытаемся остановить все катушки
            try:
                self.ser.write(b'stopall\r\n')
                time.sleep(0.5)
            except:
                pass
            
            self.ser.close()
            print("\nСоединение закрыто")

def main():
    print("="*60)
    print("УПРОЩЕННЫЙ КОНТРОЛЛЕР МАГНИТНОГО МАНИПУЛЯТОРА")
    print("="*60)
    
    # Порт по умолчанию
    port = "COM16"
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"Используется порт: {port}")
    
    # Создаем объект контроллера
    controller = SimpleMagneticManipulator(port)
    
    try:
        # Подключаемся
        if not controller.connect():
            print("Не удалось подключиться. Проверьте:")
            print("1. Подключен ли STM32 к компьютеру")
            print("2. Правильно ли выбран COM порт")
            print("3. Закрыт ли Putty и другие программы, использующие порт")
            return
        
        # Основное меню
        while True:
            print("\n" + "="*60)
            print("МЕНЮ:")
            print("1. Быстрый тест всех команд")
            print("2. Интерактивный режим (рекомендуется)")
            print("3. Тест отдельных команд")
            print("4. Выход")
            print("="*60)
            
            choice = input("\nВыберите опцию (1-4): ").strip()
            
            if choice == '1':
                controller.quick_test()
                
            elif choice == '2':
                controller.interactive_mode()
                
            elif choice == '3':
                print("\nТестирование отдельных команд")
                print("Введите команду для тестирования (например: help)")
                cmd = input("Команда: ").strip()
                if cmd:
                    controller.send_command_raw(cmd, 2)
                    
            elif choice == '4':
                print("Выход...")
                break
                
            else:
                print("Неверный выбор. Попробуйте снова.")
                
    except KeyboardInterrupt:
        print("\nПрограмма прервана пользователем")
    except Exception as e:
        print(f"\nОшибка: {e}")
    finally:
        controller.close()

if __name__ == "__main__":
    main()