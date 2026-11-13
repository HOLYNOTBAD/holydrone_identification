# USB 台架 ESC / 电机激励

`escmotor_excite.py` 是独立 Python 工具，不依赖 ROS。电脑经 USB/MAVLink 发送 PX4 执行器测试命令；原始测试指令、输出、转速、电压及电流由飞控写入 SD 卡 ULog。程序不解锁、不改变飞行模式，也不自动下载日志。

## 准备

1. 安装依赖：`python -m pip install pymavlink pyserial PyYAML`。
2. 使用已固定的台架和外部电源；先核对电机编号与 ESC 遥测索引。USB 不负责电机供电。ESC 必须提供可读取的 RPM。仅有 BDShot 转速时，设置 `voltage_source: battery`、`battery_id: 0` 和 `require_esc_current: false`，使用飞控电源测量电压；不将电池总电流当成 ESC 电流。需要 ESC 电压、电流时可选择 `voltage_source: esc` 并启用电流检查。
3. 将 [escmotor_logger_topics.txt](../scripts/escmotor/escmotor_logger_topics.txt) 复制到 SD 卡的 `etc/logging/logger_topics.txt`，保留原文件备份。此文件是台架专用；本机 PX4 分支中它会替代默认话题列表，实验结束后应恢复原配置。
4. 飞控保持未解锁，执行器测试功能需已启用且满足固件的安全开关要求。程序不修改这些参数。关闭占用 USB 串口的 QGC/MAVROS；也不要同时运行其他执行器测试程序。
5. 编辑 [escmotor.yaml](../config/escmotor/escmotor.yaml)：填写 `precheck_command`、`voltages_v`、`commands`、`voltage_tolerance_v`，核对电机/ESC 编号。模板故意不提供未知硬件的激励幅值和电压。`port: auto` 仅在发现唯一 USB 串口时有效，否则指定 `/dev/serial/by-id/...`。

接口依据本机 `PX4-Autopilot-holydrone` 的 Commander、logger 和 ESC_STATUS 实现。其他固件需先核对支持情况：本工具使用 `MAV_CMD_ACTUATOR_TEST`（310），param1 是测试值，param2 是超时秒数，param5 是从 1 开始的 MAVLink 电机功能编号。它不是 PWM 微秒值，也不能未经核对就等同于正常飞行的 `actuator_motors.control[i]`。停止通过 NaN 输出和 timeout=0 释放测试控制。

## 运行

从仓库根目录执行，输出目录必须不存在：

```bash
python scripts/escmotor/escmotor_excite.py --config config/escmotor/escmotor.yaml \
  --output data/escmotor/session_001 --check-config

python scripts/escmotor/escmotor_excite.py --config config/escmotor/escmotor.yaml \
  --output data/escmotor/session_001
```

依次进行 USB 插入确认、PX4 心跳检查、日志配置检查、短时试转、遥测检查、电源档位确认，以及各次激励确认。每个电压下按指令列表顺序执行，各指令重复 `repetitions` 次，每次均等待 Enter。输入任意非空内容或 Ctrl+C 退出。

程序拒绝接管已经正在记录文件的 logger。检查 SD 卡配置后，会重启 logger 以重新加载话题列表，使用 `logger on` 开始未解锁记录，并检查 `Full File Logging Running`。结束时执行 `logger off`；若飞控原有日志模式要求持续记录，日志可能继续，需查看飞控状态。不依赖为记录数据而解锁。

每次激励为：停止状态记录 → 指定值保持 → 释放输出 → 记录降速 → 确认 RPM 降至停止门槛。保持期间每约 0.2 s 续发一次测试请求，飞控请求超时为 0.6 s；命令拒绝、确认超时、心跳/遥测过期或发现已解锁会中止。正常停止靠显式释放，USB 断开时依赖飞控超时。试转同样需要 Enter。

## 数据与判定

- SD 卡 `.ulg`：`actuator_test`、`actuator_outputs` 的多个实例、`esc_status`、`battery_status`、命令与状态。话题配置为不额外降采样，但不能提高 ESC 自身刷新率，也不保证 SD 卡无丢样。
- `config.yaml`：本次配置快照。
- `events.jsonl`：连接、每次激励、停止、结果或异常，以及 logger 状态和 ULog 路径。包含电脑时钟和最近收到的 ESC 飞控时间；后者只是对应线索，拟合跳变时刻应使用 ULog 内的 `actuator_test.timestamp`。

预检查与正式脉冲均要求足够独立遥测样本、运行 RPM 超过门槛，并在停止后降到停止门槛；仅在 `require_esc_current: true` 时要求 ESC 电流有效且运行期间出现正值。数值有限和电流为正只证明基本回传可用，不代表传感器已标定或通道映射已证明正确。电压校验使用 `voltage_source` 指定的实测来源；激励期间允许带载压降，事件清单报告电压范围。

末段 RPM 的相对跨度用于初步稳态筛查，未稳定时标记 `steady: false`，不自动延长激励。默认 1 s 只是初始实验时长，需根据实际响应调整。这套停止—起转序列覆盖启动动态和稳态映射；正常转动工作点间的一阶时间常数，需要另行扩展平台间阶跃，不能直接将启动响应当作运行区间时间常数。

运行后应检查 ULog 中各话题实际存在、更新率、各 ESC 样本时间戳与丢包情况。当前实现在线检查的是日志文件写入状态与 MAVLink ESC 遥测，不能在线证明 ULog 已完整记录每个信号。

## Shell 入口与离线处理

三个入口统一放在 `_scripts/`，在仓库根目录依次执行：

```bash
bash _scripts/real_flight/escmotor/escmotor_excite.sh
bash _scripts/dataset/escmotor/escmotor_expdataprocess.sh
bash _scripts/real_flight/escmotor/escmotor_identification.sh
```

脚本默认使用 `holydrone_autotune` Conda 环境，所需依赖为 `pymavlink pyserial PyYAML pyulog numpy scipy`。环境名称和数据路径均在各脚本参数区修改。采集入口以前台方式保留键盘交互，自动使用新的时间戳会话目录；采集后自行下载 SD 卡 ULog。整理入口的 `SESSION` 必须指向对应会话，`ULOG` 指向该会话的日志；`OUTPUT` 为新的数据集目录。辨识入口的 `DATASET` 与该目录一致，报告路径不能已存在。

`escmotor_expdataprocess.py` 从 `actuator_test` 重建脉冲，使用逐 ESC 时间戳剔除重复遥测，并与 `events.jsonl` 按顺序、幅值及飞控时间核对。要求 `esc_rpm` 已由驱动转换为机械 RPM；电气 eRPM 不能直接输入。排除试转、不完整、反馈断采和末段不稳定的片段，每个目标电压/指令组合至少保留两个有效脉冲，最后一个用于验证。输出 `samples.csv` 与记录来源、划分和排除原因的 `dataset.yaml`。本版本保守拒绝含 ULog 丢写记录的日志。

`escmotor_identification.py` 使用训练脉冲的稳态均值建立逐工况转速表，并拟合带延迟的一阶启动响应；验证脉冲不参与参数估计。报告包含实际训练/验证电压、稳态转速、启动时间常数、延迟、验证误差和边界/采样分辨率检查。采样筛查要求时间常数至少为样本中位间隔的三倍；它只是初步筛查，不证明参数唯一。此版本不做电压外推、全域静态曲线拟合或停机响应拟合，拟合延迟也可能包含驱动与遥测影响。

## 离线验证命令

```bash
python -m unittest discover -s test -p 'test_escmotor*.py' -v
```

测试使用模拟连接，不发送真实电机指令。

电压来源为 battery 时，在线请求 BATTERY_STATUS，离线从 ULog battery_status 取最近且未过期的前值；本机 PX4 将 uORB 电池 ID 减 1 后作为 MAVLink ID，因此配置 battery_id=0 对应 ULog id=1。缺失或过期的电源数据仍会阻断，不用目标电压代替测量。
