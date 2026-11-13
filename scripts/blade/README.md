# 静态旋翼推力特性

原始表格与来源信息位于 [blade_experiment](../../data/blade/experiment/README.md)，统一索引和样本位于 [blade_dataset](../../data/blade/dataset/README.md)。`manufacturer` 是此前的厂家表格，`summer_measurement` 是暑假实测。旧 `scripts/blade/static_thrust.csv` 已移至 `data/blade/experiment/manufacturer/static_thrust.csv`。

在仓库根目录运行：

```bash
# 整理全部原始实验，只依赖 Python 标准库
python3 scripts/blade/blade_catalog.py
# 分别拟合，依赖 numpy、PyYAML、matplotlib
conda run --no-capture-output -n holydrone_autotune python scripts/blade/blade_fit.py --experiment manufacturer
conda run --no-capture-output -n holydrone_autotune python scripts/blade/blade_fit.py --experiment summer_measurement
# 查询实测范围内的推力
conda run --no-capture-output -n holydrone_autotune python scripts/blade/blade_fit.py --experiment summer_measurement --rpm 8000
```

默认拟合 `manufacturer`。也可用 `--table <CSV>` 输入自定义数据，与 `--experiment` 互斥；自定义表不会继承厂家硬件信息。`--output` 可指定输出目录，默认输出到 `data/blade/dataset/fits/<experiment_id>/`（自定义表为 `custom`）。各实验结果分别保存，同一实验重复运行更新同名结果。

程序将克重换算为牛顿 `F = thrust_g × 9.81 / 1000`，将机械 RPM 换算为角速度 `omega = rpm × 2π / 60`，以无权重、过原点最小二乘拟合：

```text
F = k_f × omega²
k_f = sum(omega² × F) / sum(omega⁴)
```

厂家表最大推力 2100 g 行缺少 RPM，保留原始行但排除拟合，不推测转速。厂家使用 5 个点，实测使用 4 个点。`lookup()` 提供 RPM 分段线性插值；查表和命令行查询只允许所选实验的有效区间，不添加虚构零点或外推。`fit_square()` 接受 RPM 和牛顿数组，返回系数、预测和误差。

每次输出 `report.yaml`（来源、原表指纹、单位、范围、排除行、系数及样本内误差）、`fit_points.csv`（实测与预测、残差、相对误差）和 `thrust_fit.png/pdf`（曲线与残差）。厂家表的 `k_f` 约为 `5.12294e-6 N/(rad/s)²`，样本内 RMSE 约 0.187 N。

两组数据分别拟合；暑假实测硬件型号未知。此模型是静态转速—推力关系，不能据此确定电机时间常数或气动反扭矩系数。电流、功率、效率和动力输出字段保留用于追溯，不作为本次拟合输入；拟合误差不是独立验证结果。

## 工作区间详测：终端输入推力

`blade_bench.py`、`blade_mavlink.py`、消息 XML 与 logger topics 从 escmotor 拷贝，保留 PX4 通信、命令确认、遥测检查、释放测试控制和可选 ULog 支持；blade 独立维护，不导入 escmotor，也未修改原实验代码。新增 `blade_excite.py` 负责静态测量流程。本机 CSV 始终保存，默认无需 ULog；`record_ulog: true` 时需按原 escmotor 方式预先安装 SD 卡日志主题配置。

先编辑 `config/blade/blade.yaml`：`commands` 必须填写实际工作区油门列表（0–1），电机/ESC 编号和 16.8 V 只是沿用当前配置，需对应本次接线。升降斜率默认每秒 2 个百分点；可根据电源调整。每轮按列表顺序执行，可填入降档点；轮次间也缓慢过渡，不逐档重新起动。

```bash
# 仅检查配置，不连接飞控；未填写 commands 时会报错
conda run --no-capture-output -n holydrone_autotune python -I scripts/blade/blade_excite.py --check-config
# 在当前终端运行
bash _scripts/real_flight/blade/blade_excite.sh
# 测量完成或中断后整理；有效的已提交点均保留
python3 scripts/blade/blade_catalog.py
# 替换为本次目录名
conda run --no-capture-output -n holydrone_autotune python scripts/blade/blade_fit.py --experiment bench_YYYYmmdd_HHMMSS
```

流程为：停止状态确认 → 从零缓升至第一档 → 等待 `settle_s` → 终端提示推力输入 → 保存 → 缓慢过渡到下一档 → 正常缓降到零并释放测试控制。斜坡按归一化油门变化率限制，并限制卡顿后的单步增量；没有闭环电流控制，不保证电源不会触发限流。

输入有限正数（单位 g，不带字符）后回车；`s` 跳过本档，`q` 在读数阶段退出，`Ctrl+C` 可在任意阶段中止。等待输入期间继续收遥测和续发当前指令。每档输入总超时默认 120 秒，输错或窗口未稳定不会重置时限。正常结束缓降；中止、超时、反馈失效或电压越限时直接释放测试输出。沿用 0.6 秒的飞控指令租期，定期续发。

提示开始后至少等待一个 `steady_window_s`（默认 2 秒），观察台架稳定再输入；提交前窗口计算 RPM 均值、标准差、极差/均值以及电压、电流均值。样本不足、反馈断采、未起转或 RPM 波动过大时拒绝该次读数并提示重新输入。推力与 RPM 的同步精度受人工读取和输入延迟限制。可选 ESC 电流全零/无效时留空，不将它解释为实测零电流。

每个会话即时保存：

- `config.yaml`、`metadata.json`：参数和来源；未知硬件留空。
- `telemetry.csv`：各阶段的原始遥测，保留主机接收单调时间、ESC 时间戳、油门和阶段。当前电压来自配置指定来源，过期时留空。
- `static_thrust.csv`：已接受的手动推力及窗口统计，每次提交刷盘；中断后仍可整理和拟合。
- `events.jsonl`：连接、斜坡、跳过、拒绝读数、保存和停转事件。
- `status.json`：`incomplete`、`aborted` 或 `complete`；完整会话可能包含跳过点，应结合事件查看。

拟合使用实际 RPM 和推力，容许带载电压的小幅波动，报告电压为有效点均值。重复 RPM 样本仍分别参与最小二乘，插值时同 RPM 节点取平均推力。不要把不同硬件或大幅不同电压下的记录拼成同一次实验。


## 四组拟合与 HTML 对比

运行以下命令，将仅厂商、仅暑假、仅今日和三组共同拟合的结果写入 `data/blade/fit/`：

```bash
conda run --no-capture-output -n holydrone_autotune python scripts/blade/blade_fit_compare.py
```

打开 [离线报告](../../data/blade/fit/index.html)，可切换查看曲线、残差、按来源的误差和逐点预测，也可下载 PNG/PDF/CSV/JSON。脚本复用 `fit_square()`，先重建 dataset，再按可拟合标记选点。联合模型每点等权，不对三个实验分别等权。

“今日”默认固定为 `bench_20261113_113812`，可用 `--today-experiment <实验编号>` 指定其他会话；联合组始终是所选的厂商、暑假和该会话三个来源。每组只在其已测 RPM 范围画曲线。`--output` 可更改报告目录。
