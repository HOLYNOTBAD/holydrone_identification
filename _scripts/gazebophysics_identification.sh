#!/usr/bin/env bash
# 离线辨识：读取独立训练/验证 ULog，生成 YAML 报告；不启动 ROS 或飞行。
# 用法：本脚本 --train train.ulg --validation validation.ulg --output report.yaml
# 支持 TRAIN、VALIDATION、OUTPUT 环境变量；额外参数原样传给处理程序。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-}"  # 可指定解释器；未安装 Conda 时使用系统 python3
CONDA_ENV="${CONDA_ENV:-holydrone_autotune}"
CONDA_EXE="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
TRAIN="${TRAIN:-}"
VALIDATION="${VALIDATION:-}"
OUTPUT="${OUTPUT:-}"
CONFIG="${CONFIG:-$ROOT/config/gazebo_physics.yaml}"

####################### 通用函数区 #######################

####################### 启动区 #######################
ARGS=(--config "$CONFIG")
[[ -z "$TRAIN" ]] || ARGS+=(--train "$TRAIN")
[[ -z "$VALIDATION" ]] || ARGS+=(--validation "$VALIDATION")
[[ -z "$OUTPUT" ]] || ARGS+=(--output "$OUTPUT")
if [[ -n "$PYTHON" ]]; then
  RUN=("$PYTHON")
elif [[ -x "$CONDA_EXE" ]]; then
  RUN=("$CONDA_EXE" run --no-capture-output -n "$CONDA_ENV" python)
else
  RUN=(python3)
fi
exec env -u PYTHONPATH "${RUN[@]}" -I "$ROOT/scripts/gazebo_physics_process.py" "${ARGS[@]}" "$@"
