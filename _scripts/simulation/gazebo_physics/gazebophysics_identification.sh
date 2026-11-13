#!/usr/bin/env bash
# 离线辨识；编辑参数区的日志和输出路径后运行，不启动 ROS 或飞行。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="holydrone_autotune"
TRAIN="$ROOT/data/gazebo_physics/train.ulg"
VALIDATION="$ROOT/data/gazebo_physics/validation.ulg"
CONFIG="$ROOT/config/gazebo_physics/gazebo_physics.yaml"
OUTPUT="$ROOT/data/gazebo_physics/report.yaml"
EXTRA_ARGS=() # 可在此指定 --train-rpm、--validation-rpm 等处理参数

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/gazebo_physics/gazebo_physics_process.py" --train "$TRAIN" --validation "$VALIDATION" \
  --config "$CONFIG" --output "$OUTPUT" "${EXTRA_ARGS[@]}"
