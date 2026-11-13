# holydrone_identification SITL 验证

本包激励阶段自行定高，回中采用 PX4 位置控制，头文件位于 `include/`。状态机负责模式交接、反馈新鲜度、激励调度与运动保护。

- `altitude_control.hpp`：高度、垂直速度反馈、有界积分、倾角补偿和推力限幅。
- `recenter_control.hpp`：仅在长偏航扫频激励期间抑制 roll/pitch 和水平漂移。
- `settling_gate.hpp`：用连续新反馈判定位置/航向回中。
- `fsm_mock_integration.py`：隔离 master 验证四类计划、扫频、推力重采集、门槛和中止；不连接真实飞控。

测试驱动会在隔离 ROS/Gazebo master 下执行模拟起飞、OFFBOARD 激励和降落。先构建并 source 工作空间，使用全新输出目录；本机 PX4 MAVLink 端口必须空闲。

```bash
source /opt/ros/noetic/setup.bash
source deployment/devel/setup.bash
python3 deployment/src/holydrone_identification/test/sitl_sequence.py \
  --kind brct1order --px4 /path/to/PX4-Autopilot --output /tmp/new-brct1order-check
```

使用 `--kind gazebo_physics` 验证物理辨识序列。

`--quick` 仅用于缩短排查，不代表完整默认配置通过。迁移前的历史结果保存在 `test/results/`；这些数据来自单次无风仿真，不代表本次修改重新执行了 SITL。


## OFFBOARD 预发送离线故障排查（2026-11-06）

`fsm_mock_integration.py` 在独立 ROS master 验证：

- 已解锁 POSCTL、FCU 状态 1 Hz、无推力回传：仍持续预发送位置目标，实际发布频率超过 100 Hz（配置 200 Hz），不等待先进入 OFFBOARD。
- 仅 IMU 时间戳超前 0.2 s 或滞后 0.2 s，而位置/速度正常：停止预发送，状态报告 IMU 新鲜度阻断；修复时间戳后无需重启即可恢复。
- 仅 IMU 回传停止：阻断后可自行恢复。
- FCU 状态超过 2 s 不更新：停止发送；恢复 1 Hz 状态后自行恢复。
- STABILIZED、未解锁和过期位置反馈不满足发送条件，测试不会自动切真实飞控模式。

完整模拟回归通过（118.451 s），没有为了通过测试放宽实飞反馈门限。此前 Gazebo 也已验证 POSCTL 预发送、OFFBOARD 进入和辨识完成，结果在 `test/results/offboard_entry_diagnostics.json`。

这些测试说明当前源码正常条件下可进入，且时间戳/反馈异常可复现无预发送；不能证明 log_105 实飞就是时间同步故障。当时 LBC 离线，未能核验机载二进制；后续验证见下节。

## LBC 旧二进制对照与诊断版部署（2026-11-06）

有线 SSH 恢复后，在 LBC 的独立 ROS master 上用 `test/prestream_probe.py` 直接运行旧 ARM 二进制和新诊断版。只发布模拟反馈，不运行 MAVROS，不连接飞控。反馈为已解锁 POSCTL、状态 1 Hz、运动反馈约 50 Hz，不提供推力回传；分别验证正常、IMU 超前 0.2 s、恢复三个阶段。

| LBC 二进制 | 正常预发送 | IMU 超前后 | 恢复后 |
| --- | --- | --- | --- |
| 原有程序 | 200 Hz | 0 Hz | 200 Hz |
| 新诊断版 | 199 Hz | 0 Hz | 200 Hz |

每阶段持续 5 s，统计后 3 s；结果分别保存在 `test/results/lbc_old_prestream.json` 和 `test/results/lbc_diagnostic_prestream.json`。旧程序本身具备预发送功能，因此不能将故障直接归因为旧程序缺少预发送。测试也没有覆盖真实串口传输或当次飞行反馈。

诊断源码已同步到 LBC，删除目标对象和可执行文件后重新编译成功，并核验 ARM ELF、诊断字符串、源码哈希和隔离运行结果。旧程序与源码备份在 LBC 的 `~/.local/state/holydrone-deploy/identification-entry-20261106/`。

- 两端诊断源码 SHA-256：`18567f725905bf9096cc2c702d12ebcd4669a13cd3a35811b7d426fa53ee8ce2`。
- 原有 LBC 二进制 SHA-256：`1aaea6ba76a481e007c1c767efc09f5733340980cc67b9da00a784accc049d19`。
- 新 LBC 二进制 SHA-256：`767029f29777d25833c8ee06f22890738b1f83780394d6f08329ca7600deded5`。

没有自动启动实飞会话、解锁或切换飞行模式。实际根因仍未确定；需要真实会话的 `command_gate`、`command_hz`、反馈年龄和飞控接收证据来区分反馈门控与传输问题。

## BRCT 三阶完整实验（2026-11-08）

自动驱动已支持 `--kind brct2order` 与 `--kind brct3order`；长实验可设置 `--sequence-timeout 900`（墙钟秒），不改变飞行控制参数。三阶采集命令：

```bash
source deployment/devel/setup.bash
source "$HOME/.local/share/gazebo_ros_ws/devel/setup.bash" --extend
/usr/bin/python3 deployment/src/holydrone_identification/test/sitl_sequence.py \
  --kind brct3order --px4 "$HOME/PX4-Autopilot-holydrone-lite" \
  --sequence-timeout 900 --output /tmp/new-brct3order-experiment
```

本次使用默认配置、3 轮、0.20/0.35/0.50 s，72 个脉冲全部完成，无中止；激励阶段 188.368 s，最大高度下降 0.181 m、水平偏移 0.734 m、倾角 8.25°。测试驱动随后降落并上锁。

原始日志经现有提取器保留 62 个片段：40 训练、22 验证。回中/交接平台插入候选列表，使相邻平台配对漏掉 8 个推力和 2 个 pitch 脉冲；0.20 s 推力未进入拟合。本次没有修改提取器或放宽拟合门槛。

三阶验证 NRMSE：roll 3.27%、pitch 2.63%、yaw 1.74%、thrust 5.28%，误差均达标；但 roll/yaw 阻尼比触及或贴近下界，整体 `passed: false`。相同 CSV 上的二阶对照 `passed: true`。实验完成与模型验证通过分别记录。

图表、参数、原始 ULog、提取审计及复现脚本见 [完整实验报告](../data/ctbr/brct3order/sitl_20261108_153607/REPORT.md)。
