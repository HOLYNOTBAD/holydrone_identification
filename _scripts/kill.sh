#!/usr/bin/env bash
# 仅停止本工作区辨识脚本登记的会话；核对用户、PID 和启动时间，避免误杀。
set -euo pipefail

####################### 参数区 #######################
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

####################### 通用函数区 #######################

####################### 启动区 #######################
exec /usr/bin/python3 - "$ROOT" <<'PY'
import json, os, pathlib, signal, sys, time
root = pathlib.Path(sys.argv[1])
def start_ticks(pid):
    try:
        fields = pathlib.Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (FileNotFoundError, ProcessLookupError):
        return None

targets = {}
# 辨识入口同时支持仿真和实飞，按会话记录找到主管脚本，交由其清理全部标签页。
for path in pathlib.Path('/tmp').glob('holydrone-identification.*/process.json'):
    try:
        if path.stat().st_uid != os.getuid():
            continue
        record = json.loads(path.read_text())
        pid = record['pid']
        if record['root'] != str(root) or start_ticks(pid) != record['start_ticks']:
            continue
        command = pathlib.Path('/proc/%d/cmdline' % pid).read_bytes().split(b'\0')
        if any(part.endswith((b'/brct1order_excite.sh', b'/gazebophysics_excite.sh')) for part in command):
            targets[pid] = record['start_ticks']
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, KeyError):
        continue

if not targets:
    print('没有正在运行的 identification 会话。')
    sys.exit(0)
for pid, ticks in targets.items():
    if start_ticks(pid) == ticks:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
print('已请求停止运行，等待清理本工作区的子进程……', flush=True)
for _ in range(400):
    if all(start_ticks(pid) != ticks for pid, ticks in targets.items()):
        print('运行已停止。')
        sys.exit(0)
    time.sleep(.1)
sys.exit('清理尚未完成，请检查该次运行日志。')
PY
