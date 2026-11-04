#!/usr/bin/env bash
# 启动物理参数激励采集会话；MODE 默认 simulation，也支持 real_flight。
# 用法：可通过 OUTPUT、CONFIG 覆盖采集前缀与激励 YAML；需要 gnome-terminal 和 Catkin 环境。
set -e

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODE="${MODE:-simulation}"  # simulation | real_flight
# 仿真运行文件位于 ${OUTPUT}_px4，每次采集换一个目录；实飞解除武装后自动下载到 ${OUTPUT}_px4。
OUTPUT="${OUTPUT:-$ROOT/src/holydrone_identification/data/gazebo_physics/flight_$(date +%Y%m%d_%H%M%S)}"
CONFIG="${CONFIG:-$ROOT/src/holydrone_identification/config/gazebo_physics.yaml}"

DOWNLOAD_ULOG=false
[[ "$MODE" != real_flight ]] || DOWNLOAD_ULOG=true

####################### 通用函数区 #######################
# 清理本次登记的 launch PID，等待标签页退出后删除 SESSION 临时目录。
cleanup() {
  trap '' INT TERM
  for index in "${!PIDS[@]}"; do
    # 关闭时标签页可能仍在创建，等它登记 PID 后再结束 launch。
    while kill -0 "${PIDS[$index]}" 2>/dev/null; do
      if [[ -s "${PID_FILES[$index]}" ]]; then
        kill -TERM "$(cat "${PID_FILES[$index]}")" 2>/dev/null || true
        break
      fi
      sleep 0.05
    done
  done
  if ((${#PIDS[@]})); then
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
  # 仅删除本次 mktemp 创建的会话目录及其中的 PID 记录。
  rm -rf -- "$SESSION"
}
# --wait 让主脚本感知标签页内 launch 的退出；每页单独加载 ROS 环境。
# 参数：标签页标题及 roslaunch 参数；后台启动并登记客户端 PID 与实际 launch PID 文件。
launch_tab() {
  local title="$1"
  shift
  # 单引号中的参数留给标签页内的 bash 展开。
  # shellcheck disable=SC2016
  gnome-terminal --tab --title="$title" --working-directory="$ROOT" --wait -- \
    env "PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot-holydrone-lite}" "MAVROS_FCU_URL=${MAVROS_FCU_URL:-/dev/ttyS0:2000000}" \
      "MAVROS_TGT_SYSTEM=${MAVROS_TGT_SYSTEM:-2}" \
      "ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}" bash -c '
      set -e
      printf "%s\n" "$$" > "$2"
      source /opt/ros/noetic/setup.bash
      source "$1/devel/setup.bash"
      # Set before the SITL logger initializes; no hardware parameter changes.
      if [[ "$3" == "holydrone" && "$4" == "sitl.launch" ]]; then
        export PX4_PARAM_SDLOG_PROFILE=17
      fi
      shift 2
      exec roslaunch "$@"
    ' bash "$ROOT" "$SESSION/$title.pid" "$@" &
  PIDS+=("$!")
  PID_FILES+=("$SESSION/$title.pid")
}

####################### 启动区 #######################
[[ "$MODE" == "simulation" || "$MODE" == "real_flight" ]] || { echo "Invalid MODE: $MODE" >&2; exit 2; }
[[ -f "$CONFIG" && -f "$ROOT/devel/setup.bash" ]] || { echo "Missing CONFIG or workspace devel/setup.bash" >&2; exit 2; }
command -v gnome-terminal >/dev/null
SESSION="$(mktemp -d /tmp/holydrone-identification.XXXXXX)"

# roslaunch 在终端服务下运行，需记录实际 PID，不能只停止 gnome-terminal 客户端。
PIDS=()
PID_FILES=()
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# kill.sh 核对工作区、主管 PID 和启动时间后，再通知本脚本清理实际 launch。
/usr/bin/python3 - "$SESSION/process.json" "$ROOT" "$$" <<'PY'
import json, pathlib, sys
path, root, pid = pathlib.Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
ticks = pathlib.Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()[19]
path.write_text(json.dumps(dict(root=root, pid=pid, start_ticks=ticks)))
PY

[[ ! -e "${OUTPUT}_px4" ]] || { echo "Use a new OUTPUT prefix: $OUTPUT" >&2; exit 2; }
RPM_OUTPUT=""
if [[ "$MODE" == "simulation" ]]; then
  RPM_OUTPUT="${OUTPUT}_motor_speed.csv"
  [[ ! -e "$RPM_OUTPUT" && ! -e "${OUTPUT}_px4" ]] || { echo "Use a new OUTPUT prefix: $OUTPUT" >&2; exit 2; }
fi

if [[ "$MODE" == "simulation" ]]; then
  launch_tab "PX4-Gazebo" holydrone sitl.launch \
    aircraft:=holydrone_lite \
    start_mavros:=true \
    "px4_runtime_dir:=${OUTPUT}_px4" \
    "world:=$ROOT/src/holydrone_ros/gazebo/worlds/empty.world" \
    spawn_z:=0.055
else
  launch_tab "MAVROS" holydrone mavros_real.launch
fi

# 手动解锁、起飞悬停并进入 OFFBOARD 后开始激励；完成后请求 POSCTL。
launch_tab "Gazebo-Physics" --wait holydrone_identification gazebo_physics.launch \
  "config:=$CONFIG" "download_ulog:=$DOWNLOAD_ULOG" "output:=$OUTPUT" "$@" "motor_speed_output:=$RPM_OUTPUT"
wait -n
