# Blade 数据集

由 `python3 scripts/blade/blade_catalog.py` 从同级 `experiment/` 完整重建。请修改原始表和元数据后重新生成，不手动修改生成 CSV。实验编号使用原始目录名；样本通过 `experiment_id` 和 `source_row` 关联来源。

- `experiments.csv`：每个实验一行，含来源类型、硬件、日期、原表路径与 SHA-256、总行数和有效点数。未知硬件和日期留空。
- `samples.csv`：统一字段的全部实验数据，保留原表数值，并新增角速度 `omega_rad_s = rpm × 2π / 60`、推力 `thrust_n = thrust_g × 9.81 / 1000`、拟合可用标记及排除原因。
- `fits/<experiment_id>/`：各实验的拟合报告、拟合点和图表，由 `blade_fit.py --experiment <experiment_id>` 生成；忽略 Git。

原始字段单位由列名标识：`throttle_percent` 为 0–100 百分数，`voltage_v` 为 V，`current_a` 为 A，`rpm` 为机械转速，`thrust_g` 为克重，`power_w` 为 W，`efficiency_g_per_w` 为 g/W。空白不是零。`fit_eligible` 仅表示该行有正转速和推力，不能代表硬件或测量质量已验证。

厂家数据 6 行中 5 行可用于拟合，范围 7600–13800 rpm；暑假实测 4 行均可拟合，范围 3157–9850 rpm。两组均为 16.8 V，来源独立；实测硬件未知，不自动合并，也不自动视为厂家的独立验证集。

新台架会话由 `scripts/blade/blade_excite.py` 创建。索引新增 `session_status` 区分完整、中断和历史原表；样本表保留人工提交时间、统计窗口、重复轮次、RPM 波动和质量字段。历史原表无这些字段，留空。中断会话已接受的测量点仍可使用，不把“会话中断”直接当作单点无效。


`overview.html` 可直接用浏览器打开，按实验筛选测量表并叠加转速—推力曲线，无需服务器或网络。`catalog.json` 保存相同的总览数据，两者随整理脚本重建。

本次 `bench_20261113_113812` 保存 20 点（油门 0.02–0.40），在 0.42 档因带载电压偏离目标而中止，0.42 和 0.44 无推力读数。4 点推力已经用户确认更正：油门 0.12、0.14、0.18、0.30 分别为 144、170、220、411 g；本次 20 点均可用于分析。更正前读数保留在 `point_reviews.json` 和原始事件日志中。遥测质量 pass 不代表人工读数正确。

原实验目录的 `point_reviews.json` 以 CSV 数据行号（不含表头，从 1 开始）索引核对记录；`pending` 或 `rejected` 会排除相应点。此文件是人工维护的审核信息，不能通过猜测覆盖原始读数。`review_status` 和 `exclusion_reason` 会进入数据集，拟合脚本同样遵守核对状态。
