# ESC / 电机实验库

在仓库根目录执行 `bash _scripts/dataset/escmotor/escmotor_catalog.sh` 增量导入；双击 `overview.html` 离线查看、筛选和叠加响应曲线。

- `experiments.csv`：一次会话一行，记录来源文件名、硬件信息、输入指纹和处理版本。硬件信息未知则留空；可在原始会话配置中补充 `motor_model`、`propeller_model`、`esc_model` 后重新导入。
- `pulses.csv`：一次激励一行，包含试转与正式实验、实际时间边界、稳态统计、质量原因和样本路径。
- `samples/*.csv`：逐脉冲原始转速点，时间以 ULog 指令启用时刻为零；窗口按会话配置取启动前与停止后，相邻脉冲窗口在间隙中点裁切，避免重复采样。
- `catalog.json`：增量导入缓存及总览数据源。请勿手动修改生成文件；更新原始数据后重新运行导入。
- `import_report.json`：导入数量及未导入原因；任何会话出错时进程返回非零，其他成功会话仍会保存。

实验编号由原始会话目录名确定，脉冲编号按该会话顺序固定。会话名应唯一且保持不变。输入指纹覆盖 ULog、config.yaml 与 events.jsonl；相同输入与处理版本直接跳过，变化时更新该会话，源目录中暂时缺失的已导入会话继续保留。每个会话需恰好一个 ULog，且事件清单中含 experiment_complete。

样本字段：`time_s`、`test_active`、`command`、`phase`、`rpm`、`voltage_v`、`supply_current_a`、`esc_current_a`。释放测试控制时 command 留空，不能等同于主动发送零指令。ESC 时间戳去重；电池反馈使用采样时刻前最近一条记录，超过配置时限则留空，不插值跨越缺失区间。CSV 电压/电流留空表示无可用值；ESC 电流全零可能意味着未回传。

供电总电流不是单电机电流。机械 RPM 以飞控驱动换算正确为前提，需核对电机磁极数。质量 pass 仅表示当前初筛通过，不证明硬件、传感器或模型已验证。ULog 丢写、缺少完成事件、样本不足、反馈断采、无效电压/转速、未起转、末段未稳定等会保留并标记；不在本库固定训练/验证划分。稳态 RPM 取保持末段配置窗口均值，mean_voltage_v 取整个保持段均值。

原始 ULog 与会话位于同级 experiment，Git 忽略；本目录内容纳入 Git。此库不等同于旧 escmotor_expdataprocess.py 输出的训练数据集，不能直接传入旧辨识器。


拟合展示入口：在仓库根目录运行 `conda run --no-capture-output -n holydrone_autotune python scripts/escmotor/escmotor_fit.py`，再打开 [escmotor_fit/index.html](../fit/index.html)。分别展示两次实验和联合数据的稳态二次映射、逐油门带延迟一阶启动响应。重复脉冲最后一次留出验证，单次脉冲只有样本内指标；该入口直接读取当前实验库，不改变旧辨识器的输入要求。
