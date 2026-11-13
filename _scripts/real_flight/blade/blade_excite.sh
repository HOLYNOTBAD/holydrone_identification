#!/usr/bin/env bash
# 推力台架交互采集；先填写 config/blade/blade.yaml，在当前终端手动输入推力。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONDA_ENV="holydrone_autotune"
CONFIG="$ROOT/config/blade/blade.yaml"
OUTPUT="$ROOT/data/blade/experiment/bench_$(date +%Y%m%d_%H%M%S)"

####################### 环境变量区 #######################
unset PYTHONPATH

####################### 通用函数区 #######################

####################### 启动区 #######################
exec conda run --no-capture-output -n "$CONDA_ENV" python -I \
  "$ROOT/scripts/blade/blade_excite.py" --config "$CONFIG" --output "$OUTPUT"
