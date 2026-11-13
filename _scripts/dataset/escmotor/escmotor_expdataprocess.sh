#!/usr/bin/env bash
# 从同次实验的 ULog 与会话清单整理 dataset；编辑下方路径后运行。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="holydrone_autotune"
ULOG="$ROOT/data/escmotor/flight.ulg"
SESSION="$ROOT/data/escmotor/session_001"
OUTPUT="$ROOT/data/escmotor/dataset"

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/escmotor/escmotor_expdataprocess.py" \
  --ulog "$ULOG" --session "$SESSION" --output "$OUTPUT"
