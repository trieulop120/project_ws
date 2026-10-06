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

        u       i       o
        ↖       ↑       ↗

        j       k       l
        ←      STOP     →

        m       ,       .
        ↙       ↓       ↘

    u : Tiến + Xoay trái
    i : Tiến
    o : Tiến + Xoay phải

    j : Xoay trái
    k : DỪNG TẤT CẢ
    l : Xoay phải

    m : Lùi + Xoay trái
    , : Lùi
    . : Lùi + Xoay phải

[Tốc độ lái]
    w / x : Tăng / Giảm tốc độ thẳng
    e / c : Tăng / Giảm tốc độ xoay

[Điều khiển Nâng / Hạ Trục Vít Me]
    h : Set Home (Hạ vít me tìm công tắc hành trình)
    z : Nhập độ cao mong muốn H<cm> (Ví dụ: 25 -> H25)
    n : Cài tốc độ nâng M<rpm> hoặc nhập 0 để DỪNG KHẨN CẤP

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

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.pub_esp32_cmd = self.create_publisher(
            String,
            '/esp32_cmd',
            10
        )

        # Tốc độ lái
        self.speed = 0.25         # m/s
        self.turn = 0.8           # rad/s

        self.MAX_SPEED = 0.37
        self.MAX_TURN = 1.5

    def stop_all(self):
        """
        Dừng robot và dừng trục nâng.
        """
        t = Twist()
        self.pub_cmd_vel.publish(t)

        # Dừng khẩn cấp lift
        self.send_esp32_str("M0")

    def pub_vel(self, vx=0.0, wz=0.0):
        """
        Publish vận tốc cho robot.
        """
        t = Twist()

        t.linear.x = float(vx)
        t.linear.y = 0.0
        t.linear.z = 0.0

        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = float(wz)

        self.pub_cmd_vel.publish(t)

    def send_esp32_str(self, cmd_str):
        """
        Gửi command String tới ESP32 thông qua /esp32_cmd.
        """
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

            # ==================================================
            # ĐIỀU KHIỂN XE - TELEOP TWIST KEYBOARD 3x3
            # ==================================================

            # Hàng trên
            # u = Tiến + Xoay trái
            if key == 'u':
                node.pub_vel(
                    vx=node.speed,
                    wz=node.turn
                )
                print(
                    f"\r[LÁI] Tiến + Xoay trái "
                    f"(v={node.speed:.2f}, w={node.turn:.2f})       ",
                    end=""
                )

            # i = Tiến thẳng
            elif key == 'i':
                node.pub_vel(
                    vx=node.speed
                )
                print(
                    f"\r[LÁI] Tiến "
                    f"({node.speed:.2f} m/s)                    ",
                    end=""
                )

            # o = Tiến + Xoay phải
            elif key == 'o':
                node.pub_vel(
                    vx=node.speed,
                    wz=-node.turn
                )
                print(
                    f"\r[LÁI] Tiến + Xoay phải "
                    f"(v={node.speed:.2f}, w={-node.turn:.2f})      ",
                    end=""
                )

            # Hàng giữa
            # j = Xoay trái
            elif key == 'j':
                node.pub_vel(
                    wz=node.turn
                )
                print(
                    f"\r[LÁI] Xoay trái "
                    f"({node.turn:.2f} rad/s)                 ",
                    end=""
                )

            # k = Dừng tất cả
            elif key == 'k' or key == ' ':
                node.stop_all()
                print(
                    "\r[HỆ THỐNG] DỪNG TẤT CẢ (Đã gửi M0)                ",
                    end=""
                )

            # l = Xoay phải
            elif key == 'l':
                node.pub_vel(
                    wz=-node.turn
                )
                print(
                    f"\r[LÁI] Xoay phải "
                    f"({node.turn:.2f} rad/s)                ",
                    end=""
                )

            # Hàng dưới
            # m = Lùi + Xoay trái
            elif key == 'm':
                node.pub_vel(
                    vx=-node.speed,
                    wz=node.turn
                )
                print(
                    f"\r[LÁI] Lùi + Xoay trái "
                    f"(v={-node.speed:.2f}, w={node.turn:.2f})      ",
                    end=""
                )

            # , = Lùi thẳng
            elif key == ',':
                node.pub_vel(
                    vx=-node.speed
                )
                print(
                    f"\r[LÁI] Lùi "
                    f"({node.speed:.2f} m/s)                     ",
                    end=""
                )

            # . = Lùi + Xoay phải
            elif key == '.':
                node.pub_vel(
                    vx=-node.speed,
                    wz=-node.turn
                )
                print(
                    f"\r[LÁI] Lùi + Xoay phải "
                    f"(v={-node.speed:.2f}, w={-node.turn:.2f})     ",
                    end=""
                )

            # ==================================================
            # TĂNG / GIẢM TỐC ĐỘ
            # ==================================================

            elif key == 'w':
                node.speed = min(
                    node.MAX_SPEED,
                    node.speed * 1.1
                )

                print(
                    f"\r[TỐC ĐỘ] Tăng speed thẳng -> "
                    f"{node.speed:.2f} m/s  ",
                    end=""
                )

            elif key == 'x':
                node.speed = max(
                    0.05,
                    node.speed * 0.9
                )

                print(
                    f"\r[TỐC ĐỘ] Giảm speed thẳng -> "
                    f"{node.speed:.2f} m/s  ",
                    end=""
                )

            elif key == 'e':
                node.turn = min(
                    node.MAX_TURN,
                    node.turn * 1.1
                )

                print(
                    f"\r[TỐC ĐỘ] Tăng speed xoay -> "
                    f"{node.turn:.2f} rad/s  ",
                    end=""
                )

            elif key == 'c':
                node.turn = max(
                    0.1,
                    node.turn * 0.9
                )

                print(
                    f"\r[TỐC ĐỘ] Giảm speed xoay -> "
                    f"{node.turn:.2f} rad/s  ",
                    end=""
                )

            # ==================================================
            # SET HOME TRỤC NÂNG
            # ==================================================

            elif key == 'h':
                node.send_esp32_str('HOME')

                print(
                    "\r[NÂNG HẠ] Đã gửi lệnh "
                    "'HOME' để tìm vị trí gốc 0 cm...",
                    end=""
                )

            # ==================================================
            # NHẬP CHIỀU CAO H<cm>
            # ==================================================

            elif key == 'z':
                termios.tcsetattr(
                    sys.stdin,
                    termios.TCSADRAIN,
                    settings
                )

                print("\n")

                try:
                    val = input(
                        ">> Nhập độ cao mong muốn "
                        "(cm) [0.0 - 80.0]: "
                    ).strip()

                    if val:
                        cmd = f"H{val}"

                        node.send_esp32_str(cmd)

                        print(
                            f"-> Đã gửi lệnh: '{cmd}'"
                        )

                except Exception as e:
                    print(
                        f"Lỗi nhập liệu: {e}"
                    )

                print(
                    "\nTiếp tục nhấn phím điều khiển..."
                )

            # ==================================================
            # CÀI TỐC ĐỘ NÂNG M<rpm>
            # n thay cho m
            # ==================================================

            elif key == 'n':
                termios.tcsetattr(
                    sys.stdin,
                    termios.TCSADRAIN,
                    settings
                )

                print("\n")

                try:
                    val = input(
                        ">> Nhập tốc độ nâng RPM "
                        "[VD: 30 hoặc 0 để DỪNG]: "
                    ).strip()

                    rpm = float(val)

                    cmd = f"M{rpm:.1f}"

                    node.send_esp32_str(cmd)

                    if rpm == 0:
                        print(
                            f"-> Đã gửi lệnh: '{cmd}' "
                            "(Dừng khẩn cấp)"
                        )
                    else:
                        print(
                            f"-> Đã gửi lệnh: '{cmd}' "
                            f"(Tốc độ nâng: {rpm:.1f} RPM)"
                        )

                except ValueError:
                    print(
                        f"-> Lỗi: '{val}' "
                        "không phải số hợp lệ!"
                    )

                except Exception as e:
                    print(
                        f"Lỗi nhập liệu: {e}"
                    )

                print(
                    "\nTiếp tục nhấn phím điều khiển..."
                )

            # ==================================================
            # CTRL+C
            # ==================================================

            elif key == '\x03':
                node.stop_all()

                print("\nĐã thoát.")

                break

    except Exception as e:
        print(e)

    finally:
        node.stop_all()

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
