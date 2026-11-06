#!/usr/bin/env python3
"""Launch hệ thống LLM điều khiển robot bằng tiếng Việt.

  ros2 launch amr_llm llm_system.launch.py                  # dry_run (an toàn)
  ros2 launch amr_llm llm_system.launch.py dry_run:=false   # chạy thật

Giao diện giọng nói nằm trong dashboard có sẵn (llm_voice.js) - sau khi
amr_web đã vá qua apply script, KHÔNG cần with_web.

Cần GEMINI_API_KEY trong môi trường (ví dụ: source ~/.gemini_env).
Nếu chưa có, launch vẫn chạy nhưng LLM sẽ báo "Thiếu GEMINI_API_KEY".
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _preflight(context, *args, **kwargs):
    """In cảnh báo ngay khi launch, thay vì để lỗi lộ ra sau khi đã nói lệnh."""
    msgs = []
    if not os.environ.get("GEMINI_API_KEY"):
        msgs.append(LogInfo(msg="[CẢNH BÁO] Chưa có GEMINI_API_KEY. "
                                "Chạy `source ~/.gemini_env` rồi launch lại "
                                "(nếu file chưa tồn tại thì tạo trước)."))
    if LaunchConfiguration("dry_run").perform(context).lower() == "false":
        msgs.append(LogInfo(msg="[CHẠY THẬT] dry_run=false - robot sẽ di chuyển. "
                                "Giữ nút dừng cứng trong tầm tay."))
    if LaunchConfiguration("with_web").perform(context).lower() == "true":
        msgs.append(LogInfo(msg="[CẢNH BÁO] with_web=true nhưng llm_web_node không "
                                "tồn tại trong repo - bỏ qua. Dùng dashboard "
                                "amr_web (llm_voice.js) thay thế."))
    return msgs


def generate_launch_description():
    dry_run = DeclareLaunchArgument(
        "dry_run", default_value="true",
        description="true: chỉ in kế hoạch, robot không chạy thật")
    require_confirm = DeclareLaunchArgument(
        "require_confirm", default_value="true",
        description="true: phải xác nhận kế hoạch trước khi chạy")
    with_web = DeclareLaunchArgument(
        "with_web", default_value="false",
        description="KHÓ DÙNG: llm_web_node không tồn tại trong repo; "
                    "dashboard amr_web (llm_voice.js) đã thay thế")

    llm_command_node = Node(
        package="amr_llm",
        executable="llm_command_node",
        name="llm_command_node",
        output="screen",
        parameters=[{
            # Ép kiểu bool tường minh: đây là tham số an toàn, không để suy đoán kiểu
            "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
            "require_confirm": ParameterValue(
                LaunchConfiguration("require_confirm"), value_type=bool),
            "backends": ["gemini"],
        }],
    )

    return LaunchDescription([
        dry_run, require_confirm, with_web,
        OpaqueFunction(function=_preflight),
        llm_command_node,
    ])
