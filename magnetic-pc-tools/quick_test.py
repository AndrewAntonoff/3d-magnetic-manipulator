# quick_test.py
import serial
import time

def test_system(port='COM16'):
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)
    
    commands = [
        ("status", 0.5),
        ("coil 0 0.05", 0.5),
        ("sensor 0", 0.5),
        ("coil 0 0.0", 0.5),
        ("testsensors", 1.0),
        ("testspi", 1.0),
    ]
    
    for cmd, wait in commands:
        print(f"\n>>> {cmd}")
        ser.write(f"{cmd}\r\n".encode())
        time.sleep(wait)
        response = ser.read_all().decode()
        print(response)
    
    ser.close()

if __name__ == "__main__":
    test_system()