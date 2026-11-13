#!/usr/bin/env bash
# 增量导入所有已完成的台架实验，生成 CSV 总表和离线 HTML 总览。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="holydrone_autotune"
SOURCE="$ROOT/data/escmotor/experiment"
OUTPUT="$ROOT/data/escmotor/dataset"

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/escmotor/escmotor_catalog.py" \
  --source "$SOURCE" --output "$OUTPUT"
