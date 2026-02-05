import serial
import time
import sys
import os
from colorama import init, Fore, Back, Style
import re

# Инициализация colorama для цветного вывода
init(autoreset=True)

class HardwareDetector:
    def __init__(self, port='COM3', baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.connected = False
        self.known_commands = []  # Список известных команд
        
    def connect(self):
        """Подключение к STM32"""
        try:
            print(f"{Fore.CYAN}🔌 Подключение к {self.port} (115200 бод)...{Style.RESET_ALL}")
            
            # Используем настройки из кода STM32
            self.ser = serial.Serial(
                port=self.port,
                baudrate=115200,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=2,
                write_timeout=2
            )
            
            # Даем время на инициализацию
            time.sleep(2)
            
            # Очищаем буфер
            if self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
            
            # Проверяем связь
            self.ser.write(b'\n')
            time.sleep(0.5)
            
            # Читаем приветственное сообщение если есть
            if self.ser.in_waiting:
                welcome = self.ser.read(self.ser.in_waiting)
                print(f"Получено: {welcome[:100]}")
            
            self.connected = True
            print(f"{Fore.GREEN}✅ Подключено к STM32 Magnetic Manipulator{Style.RESET_ALL}")
            
            # Читаем приветственное сообщение системы
            time.sleep(0.5)
            if self.ser.in_waiting:
                response = self.ser.read(self.ser.in_waiting)
                try:
                    decoded = response.decode('utf-8', errors='ignore')
                    print(f"{Fore.CYAN}Системное сообщение:{Style.RESET_ALL}")
                    print(decoded[:500])
                except:
                    print(f"{Fore.YELLOW}Получены бинарные данные{Style.RESET_ALL}")
            
            return True
            
        except serial.SerialException as e:
            error_msg = str(e)
            if "Access is denied" in error_msg:
                print(f"{Fore.RED}❌ Порт занят другим приложением{Style.RESET_ALL}")
                print(f"{Fore.YELLOW}Закройте Putty, TeraTerm или другие программы{Style.RESET_ALL}")
            elif "FileNotFoundError" in error_msg or "could not open port" in error_msg:
                print(f"{Fore.RED}❌ Порт {self.port} не найден{Style.RESET_ALL}")
                print(f"{Fore.YELLOW}Доступные порты:{Style.RESET_ALL}")
                self.list_com_ports()
            else:
                print(f"{Fore.RED}❌ Ошибка подключения: {e}{Style.RESET_ALL}")
            return False
        except Exception as e:
            print(f"{Fore.RED}❌ Критическая ошибка: {e}{Style.RESET_ALL}")
            return False
    
    def list_com_ports(self):
        """Список доступных COM портов"""
        if os.name == 'nt':  # Windows
            import winreg
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
                if ports:
                    print(f"{Fore.GREEN}Найдены порты: {', '.join(ports)}{Style.RESET_ALL}")
                else:
                    print(f"{Fore.YELLOW}COM порты не найдены{Style.RESET_ALL}")
            except:
                print(f"{Fore.YELLOW}Не удалось получить список портов{Style.RESET_ALL}")
        else:  # Linux/Mac
            import glob
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if ports:
                print(f"{Fore.GREEN}Найдены порты: {', '.join(ports)}{Style.RESET_ALL}")
    
    def send_command(self, command, wait=0.5, show_command=True):
        """Отправка команды на STM32"""
        if not self.connected or not self.ser:
            return ""
        
        if show_command:
            print(f"{Fore.CYAN}→ Отправка: '{command}'{Style.RESET_ALL}")
        
        try:
            # Очищаем входной буфер
            if self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
            
            # Отправляем команду с \r\n окончанием
            full_cmd = command + '\r\n'
            self.ser.write(full_cmd.encode('utf-8', errors='ignore'))
            
            # Ждем ответа
            time.sleep(wait)
            
            # Читаем ответ
            response = b''
            start_time = time.time()
            timeout = 3.0  # Максимальное время ожидания
            
            while time.time() - start_time < timeout:
                if self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting)
                    response += chunk
                    # Если получили данные, ждем еще немного
                    time.sleep(0.01)
                else:
                    # Если данных нет, проверяем таймаут
                    if response:
                        break
                    time.sleep(0.01)
            
            # Декодируем ответ
            if response:
                try:
                    decoded = response.decode('utf-8')
                    if show_command and decoded.strip():
                        print(f"{Fore.GREEN}← Ответ ({len(decoded.strip())} символов):{Style.RESET_ALL}")
                        lines = decoded.strip().split('\n')
                        for line in lines[:15]:  # Показываем первые 15 строк
                            if line.strip():
                                print(f"  {line.strip()}")
                        if len(lines) > 15:
                            print(f"  ... и еще {len(lines) - 15} строк")
                    return decoded
                except:
                    if show_command:
                        print(f"{Fore.YELLOW}← Получены бинарные данные: {response[:50]}...{Style.RESET_ALL}")
                    return str(response)
            
            if show_command:
                print(f"{Fore.YELLOW}⚠️  Нет ответа{Style.RESET_ALL}")
            return ""
            
        except Exception as e:
            if show_command:
                print(f"{Fore.RED}Ошибка: {e}{Style.RESET_ALL}")
            return ""
    
    def test_basic_commands(self):
        """Тестирование базовых команд из кода STM32"""
        print(f"\n{Fore.CYAN}🔍 Тестирование команд STM32...{Style.RESET_ALL}")
        
        # Команды из кода STM32
        test_commands = [
            ("help", "Справка по командам"),
            ("status", "Статус системы"),
            ("hwstatus", "Статус оборудования"),
            ("testcoils", "Тест катушек"),
            ("testsensors", "Тест датчиков"),
            ("testspi", "Тест SPI"),
            ("stopall", "Остановить все катушки"),
        ]
        
        working_commands = []
        
        for cmd, description in test_commands:
            print(f"\n{Fore.WHITE}Тест: {description} ({cmd}){Style.RESET_ALL}")
            response = self.send_command(cmd, wait=0.5, show_command=False)
            
            if response and len(response.strip()) > 0:
                print(f"  {Fore.GREEN}✅ OK{Style.RESET_ALL}")
                working_commands.append((cmd, response.strip()[:100]))
                
                # Показываем краткий ответ
                lines = response.strip().split('\n')
                for line in lines[:3]:
                    if line.strip():
                        print(f"    {line.strip()}")
            else:
                print(f"  {Fore.RED}❌ Нет ответа{Style.RESET_ALL}")
        
        self.known_commands = working_commands
        
        if working_commands:
            print(f"\n{Fore.GREEN}📋 Рабочие команды:{Style.RESET_ALL}")
            for cmd, resp in working_commands:
                print(f"  • {cmd}: {resp[:50]}...")
            return True
        else:
            print(f"{Fore.RED}❌ Нет рабочих команд{Style.RESET_ALL}")
            return False
    
    def detect_hardware(self):
        """Обнаружение оборудования с использованием команд из кода STM32"""
        print(f"\n{Fore.CYAN}🔍 Обнаружение оборудования...{Style.RESET_ALL}")
        
        if not self.connected:
            print(f"{Fore.RED}❌ Нет подключения{Style.RESET_ALL}")
            return
        
        # 1. Проверка статуса системы
        print(f"\n{Fore.WHITE}1. Проверка статуса системы:{Style.RESET_ALL}")
        response = self.send_command("status", wait=0.5, show_command=False)
        
        if response and len(response.strip()) > 0:
            print(f"  {Fore.GREEN}✅ Система отвечает{Style.RESET_ALL}")
            # Показываем ключевую информацию
            lines = response.strip().split('\n')
            for line in lines:
                if "Uptime:" in line or "Mode:" in line or "Emergency" in line:
                    print(f"    {Fore.CYAN}{line.strip()}{Style.RESET_ALL}")
        else:
            # Пробуем команду help
            response = self.send_command("help", wait=0.5, show_command=False)
            if response:
                print(f"  {Fore.GREEN}✅ Система отвечает на help{Style.RESET_ALL}")
            else:
                print(f"  {Fore.RED}❌ Система не отвечает{Style.RESET_ALL}")
                return
        
        # 2. Проверка оборудования
        print(f"\n{Fore.WHITE}2. Проверка оборудования:{Style.RESET_ALL}")
        response = self.send_command("hwstatus", wait=1.0, show_command=False)
        
        if response:
            print(f"  {Fore.GREEN}✅ Оборудование найдено{Style.RESET_ALL}")
            
            # Анализируем ответ
            lines = response.strip().split('\n')
            sensors_found = 0
            coils_status = "OK"
            
            for line in lines:
                if "SENSORS" in line or "Sensors:" in line:
                    print(f"    {Fore.YELLOW}Датчики:{Style.RESET_ALL}")
                elif "Sensor" in line and ("CONNECTED" in line or "connected" in line):
                    sensors_found += 1
                    print(f"    {Fore.GREEN}{line.strip()}{Style.RESET_ALL}")
                elif "Sensor" in line and ("NOT CONNECTED" in line or "not connected" in line):
                    print(f"    {Fore.RED}{line.strip()}{Style.RESET_ALL}")
                elif "Coils:" in line or "COILS" in line:
                    print(f"    {Fore.YELLOW}Катушки:{Style.RESET_ALL}")
                elif "Coil" in line or "C1" in line or "C2" in line:
                    if "ON" in line or "on" in line:
                        print(f"    {Fore.GREEN}{line.strip()}{Style.RESET_ALL}")
                    elif "OFF" in line or "off" in line:
                        print(f"    {Fore.BLUE}{line.strip()}{Style.RESET_ALL}")
            
            print(f"\n  {Fore.CYAN}Статистика:{Style.RESET_ALL}")
            print(f"    Датчиков найдено: {sensors_found}")
        else:
            print(f"  {Fore.YELLOW}⚠️  Не удалось получить статус оборудования{Style.RESET_ALL}")
        
        # 3. Тест датчиков
        print(f"\n{Fore.WHITE}3. Тест датчиков MLX90393:{Style.RESET_ALL}")
        response = self.send_command("testsensors", wait=2.0, show_command=False)
        
        if response:
            print(f"  {Fore.GREEN}✅ Тест датчиков выполнен{Style.RESET_ALL}")
            
            # Анализируем результаты теста
            lines = response.strip().split('\n')
            test_passed = False
            sensors_tested = 0
            
            for line in lines:
                if "SENSOR TEST SUITE" in line:
                    print(f"    {Fore.CYAN}=== ТЕСТ ДАТЧИКОВ ==={Style.RESET_ALL}")
                elif "Sensor" in line and ("CONNECTED" in line or "connected" in line):
                    sensors_tested += 1
                    print(f"    {Fore.GREEN}{line.strip()}{Style.RESET_ALL}")
                elif "Sensor" in line and ("NOT CONNECTED" in line or "not connected" in line):
                    print(f"    {Fore.RED}{line.strip()}{Style.RESET_ALL}")
                elif "Summary:" in line or "Всего:" in line:
                    test_passed = True
                elif "All sensors operational" in line or "Все датчики работают" in line:
                    print(f"    {Fore.GREEN}{line.strip()}{Style.RESET_ALL}")
            
            if sensors_tested > 0:
                print(f"    {Fore.CYAN}Протестировано датчиков: {sensors_tested}{Style.RESET_ALL}")
        else:
            print(f"  {Fore.YELLOW}⚠️  Не удалось выполнить тест датчиков{Style.RESET_ALL}")
        
        # 4. Тест катушек (безопасный)
        print(f"\n{Fore.WHITE}4. Тест сигналов катушек:{Style.RESET_ALL}")
        print(f"  {Fore.YELLOW}⚠️  Безопасный тест (только сигналы){Style.RESET_ALL}")
        
        # Сначала останавливаем все катушки
        self.send_command("stopall", wait=0.3, show_command=False)
        
        # Тестируем команду управления катушкой
        print(f"  Тест команды управления...")
        response = self.send_command("coil 1 0.05", wait=0.5, show_command=False)
        
        if response and ("Coil" in response or "coil" in response or "OK" in response):
            print(f"    {Fore.GREEN}✅ Команда управления работает{Style.RESET_ALL}")
            
            # Ждем немного и выключаем
            time.sleep(0.3)
            self.send_command("coil 1 0", wait=0.3, show_command=False)
            
            print(f"    {Fore.GREEN}✅ Катушка 1 протестирована{Style.RESET_ALL}")
        else:
            print(f"    {Fore.YELLOW}⚠️  Команда 'coil' не распознана{Style.RESET_ALL}")
            print(f"    {Fore.YELLOW}   Проверьте прошивку STM32{Style.RESET_ALL}")
        
        # 5. Тест SPI
        print(f"\n{Fore.WHITE}5. Тест SPI шины:{Style.RESET_ALL}")
        response = self.send_command("testspi", wait=1.0, show_command=False)
        
        if response:
            print(f"  {Fore.GREEN}✅ SPI тест выполнен{Style.RESET_ALL}")
            lines = response.strip().split('\n')
            for line in lines[:5]:
                if line.strip():
                    print(f"    {line.strip()}")
        else:
            print(f"  {Fore.YELLOW}⚠️  Не удалось выполнить тест SPI{Style.RESET_ALL}")
        
        # 6. Итоговая диагностика
        print(f"\n{Fore.CYAN}📊 ИТОГ ДИАГНОСТИКИ:{Style.RESET_ALL}")
        
        print(f"\n{Fore.GREEN}✅ Система STM32 Magnetic Manipulator обнаружена{Style.RESET_ALL}")
        print(f"{Fore.GREEN}✅ Используйте команды из списка для управления:{Style.RESET_ALL}")
        print(f"\n{Fore.CYAN}Основные команды:{Style.RESET_ALL}")
        print(f"  help          - Справка")
        print(f"  status        - Статус системы")
        print(f"  hwstatus      - Статус оборудования")
        print(f"  testsensors   - Тест датчиков")
        print(f"  testcoils     - Тест катушек")
        print(f"  coil N P      - Управление катушкой (N=1-12, P=-1.0 до 1.0)")
        print(f"  stopall       - Остановить все катушки")
        print(f"  monitor [ms]  - Мониторинг в реальном времени")
        print(f"  stopmonitor   - Остановить мониторинг")
    
    def interactive_setup_wizard(self):
        """Интерактивный мастер настройки"""
        print(f"\n{Fore.CYAN}🎯 МАГНИТНЫЙ МАНИПУЛЯТОР - МАСТЕР НАСТРОЙКИ{Style.RESET_ALL}")
        print(f"{Fore.CYAN}="*50 + f"{Style.RESET_ALL}")
        
        # Проверка подключения
        if not self.connected:
            if not self.connect():
                return False
        
        steps = [
            ("Проверка связи", self.step_connection),
            ("Проверка датчиков", self.step_sensors),
            ("Проверка катушек", self.step_coils),
            ("Калибровка системы", self.step_calibration),
        ]
        
        for i, (name, func) in enumerate(steps, 1):
            print(f"\n{Fore.WHITE}Шаг {i}: {name}{Style.RESET_ALL}")
            print(f"{Fore.CYAN}{'-'*30}{Style.RESET_ALL}")
            if not func():
                print(f"\n{Fore.RED}❌ Настройка прервана{Style.RESET_ALL}")
                choice = input("Повторить шаг? (y/n): ")
                if choice.lower() == 'y':
                    i -= 1  # Повторяем текущий шаг
                    continue
                return False
        
        print(f"\n{Fore.GREEN}✅ НАСТРОЙКА ЗАВЕРШЕНА УСПЕШНО!{Style.RESET_ALL}")
        return True
    
    def step_connection(self):
        """Шаг 1: Проверка связи"""
        print("Проверка связи с STM32...")
        response = self.send_command("status", wait=0.5, show_command=False)
        
        if response:
            print(f"{Fore.GREEN}✅ Связь установлена{Style.RESET_ALL}")
            
            # Показываем ключевую информацию
            lines = response.strip().split('\n')
            for line in lines:
                if "Uptime:" in line or "Version:" in line:
                    print(f"  {line.strip()}")
            
            return True
        else:
            print(f"{Fore.RED}❌ Нет связи с STM32{Style.RESET_ALL}")
            return False
    
    def step_sensors(self):
        """Шаг 2: Проверка датчиков"""
        print("Проверка датчиков MLX90393...")
        
        # Тестируем датчики
        response = self.send_command("testsensors", wait=2.0, show_command=True)
        
        if not response:
            print(f"{Fore.RED}❌ Не удалось проверить датчики{Style.RESET_ALL}")
            return False
        
        # Анализируем результаты
        sensors_connected = 0
        lines = response.strip().split('\n')
        
        for line in lines:
            if "Sensor" in line and "CONNECTED" in line:
                sensors_connected += 1
        
        if sensors_connected > 0:
            print(f"{Fore.GREEN}✅ Подключено датчиков: {sensors_connected}/8{Style.RESET_ALL}")
            return True
        else:
            print(f"{Fore.RED}❌ Датчики не обнаружены{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}Проверьте:{Style.RESET_ALL}")
            print("  1. Питание 3.3V на датчиках")
            print("  2. Подключение SPI (SCK, MISO, MOSI, CS)")
            print("  3. Правильность соединений")
            return False
    
    def step_coils(self):
        """Шаг 3: Проверка катушек"""
        print("Проверка катушек...")
        print(f"{Fore.YELLOW}⚠️  ВНИМАНИЕ: Проверка только сигнальных линий{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}   Для реального теста подключите питание драйверов{Style.RESET_ALL}")
        
        # Сначала останавливаем все катушки
        self.send_command("stopall", wait=0.3, show_command=False)
        
        # Тест команды управления
        print("\nТест команды управления катушкой 1...")
        response = self.send_command("coil 1 0.05", wait=0.5, show_command=True)
        
        if response and ("Coil" in response or "coil" in response or "OK" in response):
            print(f"{Fore.GREEN}✅ Команда управления работает{Style.RESET_ALL}")
            
            # Ждем 0.5 секунды
            print("Катушка 1 включена на 5% мощности (сигнал)...")
            time.sleep(0.5)
            
            # Выключаем
            self.send_command("coil 1 0", wait=0.3, show_command=False)
            print("Катушка 1 выключена")
            
            return True
        else:
            print(f"{Fore.RED}❌ Команда управления не работает{Style.RESET_ALL}")
            return False
    
    def step_calibration(self):
        """Шаг 4: Калибровка системы"""
        print("Калибровка системы...")
        print(f"{Fore.YELLOW}⚠️  Уберите все магниты из зоны датчиков{Style.RESET_ALL}")
        
        choice = input("Начать калибровку? (y/n): ")
        if choice.lower() != 'y':
            print("Калибровка пропущена")
            return True
        
        # Запускаем калибровку
        response = self.send_command("calibrate", wait=3.0, show_command=True)
        
        if response:
            print(f"{Fore.GREEN}✅ Калибровка завершена{Style.RESET_ALL}")
            return True
        else:
            print(f"{Fore.YELLOW}⚠️  Не удалось выполнить калибровку{Style.RESET_ALL}")
            return False
    
    def quick_test(self):
        """Быстрый тест системы"""
        print(f"\n{Fore.CYAN}⚡ БЫСТРЫЙ ТЕСТ СИСТЕМЫ{Style.RESET_ALL}")
        
        if not self.connected:
            if not self.connect():
                return False
        
        tests = [
            ("Проверка связи", "status", 0.5),
            ("Проверка оборудования", "hwstatus", 1.0),
            ("Тест датчиков", "testsensors", 2.0),
            ("Тест катушек", "testcoils", 1.0),
            ("Тест SPI", "testspi", 1.0),
        ]
        
        all_ok = True
        
        for test_name, command, wait_time in tests:
            print(f"\n{test_name}...")
            response = self.send_command(command, wait=wait_time, show_command=False)
            
            if response and len(response.strip()) > 0:
                print(f"{Fore.GREEN}✅ OK{Style.RESET_ALL}")
                
                # Показываем краткую информацию
                lines = response.strip().split('\n')
                info_lines = []
                for line in lines:
                    if any(keyword in line for keyword in ["Uptime:", "Sensor", "Coil", "SPI", "connected", "CONNECTED", "OK"]):
                        info_lines.append(line.strip())
                
                for line in info_lines[:3]:
                    print(f"  {line}")
            else:
                print(f"{Fore.RED}❌ FAIL{Style.RESET_ALL}")
                all_ok = False
        
        if all_ok:
            print(f"\n{Fore.GREEN}✅ ВСЕ ТЕСТЫ ПРОЙДЕНЫ{Style.RESET_ALL}")
        else:
            print(f"\n{Fore.YELLOW}⚠️  НЕКОТОРЫЕ ТЕСТЫ НЕ ПРОШЛИ{Style.RESET_ALL}")
        
        return all_ok
    
    def manual_command_mode(self):
        """Ручной ввод команд"""
        if not self.connected:
            if not self.connect():
                print(f"{Fore.RED}❌ Невозможно войти в ручной режим без подключения{Style.RESET_ALL}")
                return
        
        print(f"\n{Fore.CYAN}🎮 РУЧНОЙ РЕЖИМ УПРАВЛЕНИЯ{Style.RESET_ALL}")
        print(f"{Fore.CYAN}="*50 + f"{Style.RESET_ALL}")
        print("Доступные команды (из кода STM32):")
        print("  help          - Справка по командам")
        print("  status        - Статус системы")
        print("  hwstatus      - Статус оборудования")
        print("  testcoils     - Тест катушек")
        print("  testsensors   - Тест датчиков")
        print("  testspi       - Тест SPI")
        print("  calibrate     - Калибровка датчиков")
        print("  coil N P      - Управление катушкой (N=1-12, P=-1.0 до 1.0)")
        print("  stopall       - Остановить все катушки")
        print("  monitor [ms]  - Мониторинг в реальном времени")
        print("  stopmonitor   - Остановить мониторинг")
        print("  testall       - Запустить все тесты")
        print("  emergency     - Аварийная остановка")
        print("  reset         - Перезагрузка системы")
        print("  exit          - Выйти из ручного режима")
        print(f"\n{Fore.YELLOW}Текущий порт: {self.port}, скорость: {self.baud}{Style.RESET_ALL}")
        
        command_history = []
        
        while True:
            try:
                cmd = input(f"\n{Fore.GREEN}STM32@{self.port}> {Style.RESET_ALL}").strip()
                
                if not cmd:
                    continue
                
                if cmd.lower() == 'exit':
                    print(f"{Fore.CYAN}Выход из ручного режима{Style.RESET_ALL}")
                    break
                
                # Отправляем команду устройству
                command_history.append(cmd)
                response = self.send_command(cmd, wait=1.0, show_command=True)
                
                # Сохраняем успешные команды
                if response and len(response.strip()) > 0:
                    cmd_exists = False
                    for known_cmd, _ in self.known_commands:
                        if known_cmd == cmd:
                            cmd_exists = True
                            break
                    
                    if not cmd_exists:
                        self.known_commands.append((cmd, response.strip()[:100]))
            
            except KeyboardInterrupt:
                print(f"\n{Fore.YELLOW}Прервано (Ctrl+C){Style.RESET_ALL}")
                continue
            except Exception as e:
                print(f"{Fore.RED}Ошибка: {e}{Style.RESET_ALL}")
    
    def close(self):
        """Закрытие соединения"""
        if self.ser and self.ser.is_open:
            # Выключаем все катушки перед закрытием
            try:
                self.send_command("stopall", wait=0.1, show_command=False)
                time.sleep(0.1)
            except:
                pass
            
            self.ser.close()
            self.connected = False
            print(f"{Fore.CYAN}📴 Соединение закрыто{Style.RESET_ALL}")

def main():
    """Главная функция"""
    print(f"{Fore.CYAN}="*60 + f"{Style.RESET_ALL}")
    print(f"{Fore.CYAN}         МАГНИТНЫЙ МАНИПУЛЯТОР - ДИАГНОСТИКА v3.0{Style.RESET_ALL}")
    print(f"{Fore.CYAN}   (совместимость с прошивкой STM32 Magnetic Manipulator){Style.RESET_ALL}")
    print(f"{Fore.CYAN}="*60 + f"{Style.RESET_ALL}")
    
    # Определение COM порта
    port = None
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Используется порт: {port}")
    else:
        # Автоматический поиск COM портов
        if os.name == 'nt':  # Windows
            import winreg
            try:
                key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
                ports_found = []
                i = 0
                while True:
                    try:
                        val = winreg.EnumValue(key, i)
                        ports_found.append(val[1])
                        i += 1
                    except WindowsError:
                        break
                
                if ports_found:
                    print(f"Найдены порты: {', '.join(ports_found)}")
                    
                    # Предлагаем выбрать порт
                    print(f"\n{Fore.YELLOW}Выберите порт:{Style.RESET_ALL}")
                    for idx, p in enumerate(ports_found, 1):
                        print(f"  {idx}. {p}")
                    
                    try:
                        choice = int(input(f"\n{Fore.WHITE}Номер порта (1-{len(ports_found)}): {Style.RESET_ALL}"))
                        if 1 <= choice <= len(ports_found):
                            port = ports_found[choice - 1]
                        else:
                            port = ports_found[-1]
                    except:
                        port = ports_found[-1]
                    
                    print(f"Используется порт: {port}")
                else:
                    print(f"{Fore.YELLOW}⚠️  COM порты не найдены{Style.RESET_ALL}")
                    port = "COM16"
                    print(f"Используется порт по умолчанию: {port}")
            except:
                print(f"{Fore.YELLOW}⚠️  Не удалось определить COM порты{Style.RESET_ALL}")
                port = "COM16"
                print(f"Используется порт по умолчанию: {port}")
        else:  # Linux/Mac
            import glob
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if ports:
                port = ports[0]
                print(f"Автоопределение: найден порт {port}")
            else:
                port = "/dev/ttyUSB0"
                print(f"Используется порт по умолчанию: {port}")
    
    detector = HardwareDetector(port, baud=115200)
    
    try:
        while True:
            print(f"\n{Fore.WHITE}МЕНЮ:{Style.RESET_ALL}")
            print(f"1. {Fore.CYAN}Автоматическое обнаружение оборудования{Style.RESET_ALL}")
            print(f"2. {Fore.GREEN}Мастер настройки (рекомендуется){Style.RESET_ALL}")
            print(f"3. {Fore.YELLOW}Быстрый тест системы{Style.RESET_ALL}")
            print(f"4. {Fore.MAGENTA}Ручной ввод команд (основной режим){Style.RESET_ALL}")
            print(f"5. {Fore.BLUE}Тестирование всех команд{Style.RESET_ALL}")
            print(f"6. {Fore.RED}Выход{Style.RESET_ALL}")
            
            choice = input(f"\n{Fore.WHITE}Выберите опцию (1-6): {Style.RESET_ALL}").strip()
            
            if choice == '1':
                if detector.connect():
                    detector.detect_hardware()
            
            elif choice == '2':
                detector.interactive_setup_wizard()
            
            elif choice == '3':
                detector.quick_test()
            
            elif choice == '4':
                detector.manual_command_mode()
            
            elif choice == '5':
                if detector.connect():
                    detector.test_basic_commands()
            
            elif choice == '6':
                print(f"{Fore.CYAN}Выход из программы...{Style.RESET_ALL}")
                break
            
            else:
                print(f"{Fore.RED}Неверный выбор! Попробуйте еще раз.{Style.RESET_ALL}")
    
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Программа прервана пользователем{Style.RESET_ALL}")
    except Exception as e:
        print(f"{Fore.RED}Критическая ошибка: {e}{Style.RESET_ALL}")
        import traceback
        traceback.print_exc()
    finally:
        detector.close()

if __name__ == "__main__":
    # Проверка установки colorama
    try:
        import colorama
    except ImportError:
        print("Установите colorama: pip install colorama")
        sys.exit(1)
    
    # Проверка установки pyserial
    try:
        import serial
    except ImportError:
        print("Установите pyserial: pip install pyserial")
        sys.exit(1)
    
    main()