#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

banner = """
==================================================
  BẢNG ĐIỀU KHIỂN ROBOT AMR (TÍCH HỢP ESP32 PROTOCOL)
==================================================
[Lái bánh xe -> /cmd_vel]
    i : Tiến            , : Lùi
    j : Xoay trái       l : Xoay phải
    w / x : Tăng / Giảm tốc độ thẳng
    e / c : Tăng / Giảm tốc độ xoay
    k / Space : DỪNG TẤT CẢ (Gửi M0 ngắt trục nâng)

[Điều khiển Nâng / Hạ Trục Vít Me]
    h : Set Home (Hạ vít me tìm công tắc hành trình)
    z : Nhập độ cao mong muốn H<cm> (Ví dụ: 25 -> H25)
    m : Cài tốc độ nâng M<rpm> hoặc nhập 0 để DỪNG KHẨN CẤP

[Thoát]
    Ctrl+C : Thoát chương trình
==================================================
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopAMR(Node):
    def __init__(self):
        super().__init__('teleop_amr')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_esp32_cmd = self.create_publisher(String, '/esp32_cmd', 10)

        self.speed = 0.25         # Tốc độ thẳng (m/s)
        self.turn = 0.8           # Tốc độ xoay (rad/s)
        self.MAX_SPEED = 0.37
        self.MAX_TURN = 1.5

    def stop_all(self):
        t = Twist()
        self.pub_cmd_vel.publish(t)
        self.send_esp32_str("M0") # Dừng khẩn cấp trục nâng

    def pub_vel(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub_cmd_vel.publish(t)

    def send_esp32_str(self, cmd_str):
        s = String()
        s.data = str(cmd_str)
        self.pub_esp32_cmd.publish(s)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopAMR()

    print(banner)

    try:
        while rclpy.ok():
            key = get_key(settings)
            
            # Điều khiển xe di chuyển
            if key == 'i':
                node.pub_vel(vx=node.speed)
                print(f"\r[LÁI] Tiến ({node.speed:.2f} m/s)             ", end="")
            elif key == ',':
                node.pub_vel(vx=-node.speed)
                print(f"\r[LÁI] Lùi ({node.speed:.2f} m/s)              ", end="")
            elif key == 'j':
                node.pub_vel(wz=node.turn)
                print(f"\r[LÁI] Xoay Trái ({node.turn:.2f} rad/s)         ", end="")
            elif key == 'l':
                node.pub_vel(wz=-node.turn)
                print(f"\r[LÁI] Xoay Phải ({node.turn:.2f} rad/s)        ", end="")

            # Tăng / Giảm tốc độ
            elif key == 'w':
                node.speed = min(node.MAX_SPEED, node.speed * 1.1)
                print(f"\r[TỐC ĐỘ] Tăng speed thẳng -> {node.speed:.2f} m/s  ", end="")
            elif key == 'x':
                node.speed = max(0.05, node.speed * 0.9)
                print(f"\r[TỐC ĐỘ] Giảm speed thẳng -> {node.speed:.2f} m/s  ", end="")
            elif key == 'e':
                node.turn = min(node.MAX_TURN, node.turn * 1.1)
                print(f"\r[TỐC ĐỘ] Tăng speed xoay -> {node.turn:.2f} rad/s  ", end="")
            elif key == 'c':
                node.turn = max(0.1, node.turn * 0.9)
                print(f"\r[TỐC ĐỘ] Giảm speed xoay -> {node.turn:.2f} rad/s  ", end="")

            # Dừng tất cả
            elif key == 'k' or key == ' ':
                node.stop_all()
                print("\r[HỆ THỐNG] DỪNG TẤT CẢ (Đã gửi M0)                ", end="")

            # Set Home Trục Nâng
            elif key == 'h':
                node.send_esp32_str('HOME')
                print("\r[NÂNG HẠ] Đã gửi lệnh 'HOME' để tìm vị trí gốc 0 cm...", end="")

            # Nhập chiều cao nâng H<cm>
            elif key == 'z':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                print("\n")
                try:
                    val = input(">> Nhập độ cao mong muốn (cm) [0.0 - 80.0]: ").strip()
                    if val:
                        cmd = f"H{val}"
                        node.send_esp32_str(cmd)
                        print(f"-> Đã gửi lệnh: '{cmd}'")
                except Exception as e:
                    print(f"Lỗi nhập liệu: {e}")
                print("\nTiếp tục nhấn phím điều khiển...")

            # Cài tốc độ nâng M<rpm> hoặc dừng M0
            elif key == 'm':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                print("\n")
                try:
                    val = input(">> Nhập tốc độ nâng RPM [VD: 30 hoặc 0 để DỪNG]: ").strip()
                    rpm = float(val)
                    cmd = f"M{rpm:.1f}"
                    node.send_esp32_str(cmd)
                    if rpm == 0:
                        print(f"-> Đã gửi lệnh: '{cmd}' (Dừng khẩn cấp)")
                    else:
                        print(f"-> Đã gửi lệnh: '{cmd}' (Tốc độ nâng: {rpm:.1f} RPM)")
                except ValueError:
                    print(f"-> Lỗi: '{val}' không phải số hợp lệ!")
                except Exception as e:
                    print(f"Lỗi nhập liệu: {e}")
                print("\nTiếp tục nhấn phím điều khiển...")

            elif key == '\x03': # Ctrl+C
                node.stop_all()
                print("\nĐã thoát.")
                break
    except Exception as e:
        print(e)
    finally:
        node.stop_all()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()