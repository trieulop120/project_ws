# amr_llm

LLM command node cho AMR: ra lệnh bằng tiếng Việt (voice/text) qua dashboard,
kế hoạch JSON được validate rồi gọi các service/action có sẵn.

## Cấu trúc
- `amr_llm/llm_command_node.py` — node ROS (topics `/llm/command`, `/llm/plan`,
  `/llm/status`, `/llm/confirm`, `/llm/set_location`; service `llm/stop`)
- `amr_llm/llm_logic.py` — logic thuần (KHÔNG import ROS), test được
- `amr_llm/llm_bridge.py` — cầu nối dashboard (amr_web) <-> llm_command_node
- `launch/llm_system.launch.py` — launch hệ thống
- `scripts/start_llm.sh` — script khởi động nhanh
- `test/test_llm_logic.py` — test đơn vị (chạy không cần ROS)

## Build
    colcon build --packages-select amr_llm --symlink-install
    source install/setup.bash

## Chạy
    ros2 launch amr_llm llm_system.launch.py                 # dry_run:=true (mặc định, an toàn)
    ros2 launch amr_llm llm_system.launch.py dry_run:=false   # CHẠY THẬT

## Test
    python3 src/amr_llm/test/test_llm_logic.py
