#!/usr/bin/env bash
# Bật node LLM (llm_command_node). Giao diện giọng nói nằm trong dashboard có sẵn (llm_voice.js).
#
#   ./start_llm.sh                  # an toàn: dry_run (robot KHÔNG chạy thật)
#   DRY_RUN=false ./start_llm.sh    # chạy thật
#
# Cần: ~/.gemini_env chứa dòng  export GEMINI_API_KEY="..."
#
# Giao diện web riêng (llm_web_node.py) KHÔNG tồn tại trong repo - dùng
# dashboard amr_web (đã có llm_voice.js sau khi áp patch tích hợp).

WS="${WS:-$HOME/project_ws}"
PKG="$WS/src/amr_llm/amr_llm"
DRY_RUN="${DRY_RUN:-true}"

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
[ -f "$HOME/.gemini_env" ] && source "$HOME/.gemini_env"

if [ -z "$GEMINI_API_KEY" ]; then
  echo "Thiếu GEMINI_API_KEY. Tạo file ~/.gemini_env với dòng: export GEMINI_API_KEY=\"...\"" >&2
  exit 1
fi

echo "== LLM node | dry_run=$DRY_RUN =="
[ "$DRY_RUN" = "false" ] && echo "!! CHẾ ĐỘ CHẠY THẬT - giữ nút dừng cứng trong tầm tay !!"

python3 "$PKG/llm_command_node.py" --ros-args \
  -p dry_run:="$DRY_RUN" -p require_confirm:=true -p backends:="['gemini']" &
PIDS="$!"

trap 'kill $PIDS 2>/dev/null; wait 2>/dev/null' INT TERM EXIT
wait
