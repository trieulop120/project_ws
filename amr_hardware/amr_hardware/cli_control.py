#!/usr/bin/env python3
# [AUDIT_FIX]: Added argparse to allow specifying serial port as command line argument
import serial
import time
import threading
import sys
import argparse

# Parse command line arguments for serial port
parser = argparse.ArgumentParser(description='AMR Direct Serial Control')
parser.add_argument('port', nargs='?', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
args = parser.parse_args()

try:
    ser = serial.Serial(args.port, 115200, timeout=0.1)
except Exception as e:
    print(f"Lỗi mở cổng Serial: {e}")
    sys.exit(1)

print("Đang chờ ESP32 khởi động hoàn tất (3 giây)...")
time.sleep(3) 

# Luồng đọc phản hồi từ ESP32
def read_serial():
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Ẩn bớt gói tin PKT liên tục để không trôi màn hình nhập lệnh
                    if not line.startswith("PKT"):
                        print(f"\n[ESP32]: {line}\n> ", end="")
        except Exception:
            break

t = threading.Thread(target=read_serial, daemon=True)
t.start()

print("\n=== CHƯƠNG TRÌNH ĐIỀU KHIỂN ROBOT TRỰC TIẾP ===")
print("Gõ lệnh và bấm Enter: 'home', 'H20', 'L30', 'R30', 'M0', 'stream', 'exit'")

try:
    while True:
        cmd = input("> ")
        if cmd.strip().lower() == "exit":
            break
        if cmd.strip():
            ser.write((cmd.strip() + "\n").encode('utf-8'))
except KeyboardInterrupt:
    pass
finally:
    ser.write(b"M0\nL0\nR0\n")
    ser.close()
    print("\nĐã dừng động cơ và ngắt kết nối an toàn!")
