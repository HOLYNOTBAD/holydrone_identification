#!/usr/bin/env bash
# 离线辨识；编辑参数区的日志和输出路径后运行，不启动 ROS 或飞行。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="holydrone_autotune"
ULOG="$ROOT/data/excitation/experiment/SESSION/flight.ulg"
OUTPUT="$ROOT/data/ctbr/brct2order/report.yaml"

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/ctbr/brct2order_process.py" --ulog "$ULOG" --output "$OUTPUT"
