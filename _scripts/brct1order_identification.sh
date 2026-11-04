#!/usr/bin/env bash
# 离线辨识：从一次 ULog 按完整脉冲划分训练/验证，生成 YAML 报告；不启动 ROS 或飞行。
# 用法：本脚本 --ulog flight.ulg --output report.yaml
# 支持 ULOG、TRAIN、VALIDATION、OUTPUT 环境变量；额外参数原样传给处理程序。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-}"  # 可指定解释器；未安装 Conda 时使用系统 python3
CONDA_ENV="${CONDA_ENV:-holydrone_autotune}"
CONDA_EXE="${CONDA_EXE:-$HOME/miniforge3/bin/conda}"
ULOG="${ULOG:-}"  # 单次 excite 的完整日志
TRAIN="${TRAIN:-}"  # 兼容两航次输入
VALIDATION="${VALIDATION:-}"
OUTPUT="${OUTPUT:-}"

####################### 通用函数区 #######################

####################### 启动区 #######################
ARGS=()
[[ -z "$ULOG" ]] || ARGS+=(--ulog "$ULOG")
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
exec env -u PYTHONPATH "${RUN[@]}" -I "$ROOT/scripts/brct1order_process.py" "${ARGS[@]}" "$@"
