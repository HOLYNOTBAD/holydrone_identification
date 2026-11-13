#!/usr/bin/env bash
# 用整理好的 dataset 离线辨识；编辑路径后运行，不连接飞控。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="holydrone_autotune"
DATASET="$ROOT/data/escmotor/dataset_001"
OUTPUT="$ROOT/data/escmotor/report_001.yaml"
MAX_NRMSE=0.20

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/escmotor/escmotor_identification.py" \
  --dataset "$DATASET" --output "$OUTPUT" --max-nrmse "$MAX_NRMSE"
