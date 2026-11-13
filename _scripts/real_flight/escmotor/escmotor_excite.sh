#!/usr/bin/env bash
# USB 台架数据采集；先填写 config/escmotor/escmotor.yaml，在当前终端按提示操作。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="holydrone_autotune"
CONFIG="$ROOT/config/escmotor/escmotor.yaml"
OUTPUT="$ROOT/data/escmotor/experiment/16.8v_$(date +%Y%m%d_%H%M%S)"

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/escmotor/escmotor_excite.py" --config "$CONFIG" --output "$OUTPUT"
