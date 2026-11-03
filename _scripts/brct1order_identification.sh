#!/usr/bin/env bash
# 离线辨识：读取独立训练/验证 ULog，生成 YAML 报告；不启动 ROS 或飞行。
# 用法：本脚本 --train train.ulg --validation validation.ulg --output report.yaml
# 支持 TRAIN、VALIDATION、OUTPUT 环境变量；额外参数原样传给处理程序。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_ENV="${CONDA_ENV:-holydrone_autotune}"
CONDA_EXE="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
TRAIN="${TRAIN:-}"
VALIDATION="${VALIDATION:-}"
OUTPUT="${OUTPUT:-}"

####################### 通用函数区 #######################

####################### 启动区 #######################
[[ -x "$CONDA_EXE" ]] || { echo "Conda executable not found: $CONDA_EXE" >&2; exit 1; }
ARGS=()
[[ -z "$TRAIN" ]] || ARGS+=(--train "$TRAIN")
[[ -z "$VALIDATION" ]] || ARGS+=(--validation "$VALIDATION")
[[ -z "$OUTPUT" ]] || ARGS+=(--output "$OUTPUT")
exec env -u PYTHONPATH "$CONDA_EXE" run --no-capture-output -n "$CONDA_ENV" \
  python -I "$ROOT/scripts/brct1order_process.py" "${ARGS[@]}" "$@"
