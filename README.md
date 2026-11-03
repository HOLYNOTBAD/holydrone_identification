# holydrone_identification

ROS 1 辨识包，包含两条独立流程；飞行器、Gazebo、PX4 和 MAVROS 启动资源位于 `holydrone` 包。

| 流程            | 辨识对象                                  | 主要输出                                                   |
| --------------- | ----------------------------------------- | ---------------------------------------------------------- |
| BRCT 一阶响应   | 角速度/总推力指令到实测响应的局部等效动态 | 四通道时间常数、纯时延、增益、偏置，以及悬停推力           |
| Gazebo 物理辨识 | 四电机指令到推力和刚体转动的物理模型      | 共同电机时间常数、推力系数、对角惯量、偏航力矩系数、角阻尼 |

两者均使用独立训练、验证 ULog，只输出 YAML 报告，不自动修改 PX4 参数或 Gazebo SDF。
`holydrone_autotune` 的角速度环候选参数试验是另一套流程。

## 1. BRCT 一阶响应辨识

实现：[激励节点](src/brct1order_excite.cpp) → [日志处理](scripts/brct1order_process.py) → [拟合与验证](src/response.py)。

### 激励

按 `roll → pitch → yaw → thrust` 依次激励，每次只改变一个通道。
当前 [brct1order.yaml](config/brct1order.yaml)：roll/pitch 为 ±0.20 rad/s，yaw 为 ±0.25 rad/s，推力为每组基准 ±0.015。
统一 [identification_fsm.cpp](src/identification_fsm.cpp) 以 200 Hz 调度：Position 悬停采集推力 → Offboard 锁定高度和航向 → 回中 → 正 0.35 s → 负 0.35 s → 回中 → 下一组。重复 3 轮，共 12 组、24 个脉冲。

回中必须同时满足高度误差 ≤0.2 m、XY 偏移 ≤0.75 m、速度 ≤0.5 m/s、机体角速度模长 ≤0.15 rad/s、倾角 ≤8°、航向误差 ≤5°，持续 0.2 s；8 s 未恢复则中止。在线不自动解锁或起飞，人工退出立即停止，完成后请求 POSCTL。
角速度激励使用高度闭环及倾角补偿；推力双脉冲期间冻结闭环推力基准，回中时恢复高度反馈。
正常推力采集新鲜度为 0.5 s；PX4 模式 HEARTBEAT 为 1 Hz，进入 Offboard 的缓存交接最多允许 1.5 s，且要求模式、位姿和速度反馈有效，不能无期限使用旧推力。

### 数据与数学方法

读取 `vehicle_rates_setpoint`（CTBR设定值)、`vehicle_angular_velocity`（BR实际)、`sensor_combined`（CT实际) 和`vehicle_control_mode`，保留已解锁 OFFBOARD 数据，统一到机体 FLU 坐标系。指令按零阶保持对齐，Z 轴比力线性插值到角速度采样时刻。根据指令识别双脉冲，前后各扩展至多 0.25 s，过滤断采、无效或激励不足的片段。
离线不读取 ROS 阶段状态，也不核验是否完成全部 3 轮。

四通道分别拟合带纯时延的一阶模型：

$$
\tau\dot y(t)+y(t)=K u(t-d)+b
$$

角速度通道为“角速度指令 → 同轴实测角速度”，包含当前 PX4 角速度闭环效果；
推力通道为“归一化总推力 → 机体 Z 比力 $f_z$”。$f_z$ 在水平静态悬停时约为
9.81 m/s²，不是扣除重力后的世界系竖直加速度。

`predict()` 在延迟指令的每个恒定区间内精确积分：

$$
\hat y(t+\Delta t)=e^{-\Delta t/\tau}\hat y(t)
 +(1-e^{-\Delta t/\tau})(K u(t-d)+b)
$$

每组以首个实测输出初始化，随后递推预测。`least_squares` 使用 `soft_l1` 鲁棒损失，
从三个时延初值 0.005、0.03、0.08 s 优化，按训练 MSE 选优。
搜索范围为 $\tau\in[0.003,0.5]$ s、$d\in[0,0.15]$ s、$K\in[0.01,100]$、$b\in[-20,20]$。

### 输出与判定

每通道输出 `tau_s`、`delay_s`、`gain`、`bias`，以及训练/验证误差、边界标志和
`validated_input_range`（训练、验证输入区间交集）。角速度增益无量纲，偏置为 rad/s；
推力增益单位为 m/s² / 单位归一化推力，偏置为 m/s²。
由推力稳态模型推算 `hover_thrust`：

$$
u_{\mathrm{hover}}=(9.81-b_T)/K_T
$$

固定训练参数，在独立日志上验证。NRMSE 为 RMSE 除以实测输出的 P95−P5 跨度：

| 验证条件     | 角速度        | 推力         |
| ------------ | ------------- | ------------ |
| NRMSE        | ≤ 0.20       | ≤ 0.20      |
| RMSE         | ≤ 0.15 rad/s | ≤ 1.0 m/s² |
| 输出 P95−P5 | > 0.1 rad/s   | > 0.3 m/s²  |

还要求参数未触及代码检查的边界、共同输入范围非空，角速度范围严格跨零。
整体通过还要求悬停推力严格位于 (0, 1) 和推力共同输入范围内。
这是局部响应近似，不能分离出惯量、电机物理参数、PID 或轴间耦合。

## 2. Gazebo 物理辨识

实现：[gazebo_physics_excite.cpp](src/gazebo_physics_excite.cpp) 发布激励，
[gazebo_physics_process.py](scripts/gazebo_physics_process.py) 完成数据处理和参数拟合。

### 激励

每轮依次执行 `thrust → roll → pitch → yaw → coupled`。
当前 [gazebo_physics.yaml](config/gazebo_physics.yaml) 使用 roll/pitch ±0.30 rad/s、yaw ±0.25 rad/s、推力增量 ±0.025。每组由连续正负双脉冲组成，`pulse_durations_s: [0.20, 0.35, 0.50]` 提供不同时间尺度。若按一对正负平台定义一个方波周期，对应 2.5、1.43、1 Hz；组间回中会打断周期，这不是连续扫频。
每轮四个单通道各使用三种时长，共 12 组；另外增加两轴组合 6 组、三轴组合 4 组，耦合组轮换时长。重复 2 轮，共 44 组、88 个脉冲。每组后经统一 FSM 回中，因此总时长取决于恢复过程。去掉时长列表时兼容原来的 24 组计划。

### 数据与已知量

在线仍发送角速度/总推力指令，经 PX4 控制与分配后产生电机指令。
离线读取 `actuator_motors.control[0..3]`，按 ULog 的 `THR_MDL_FAC` 反解 PX4 推力曲线，再按 SITL 1000..2000 PWM 量化得到 Gazebo 速度指令。要求电机直接映射、非可逆，日志电机采样中位间隔不大于 20 ms。不能直接把分配器推力指令当成转速指令。
输出观测取自 `vehicle_angular_velocity` 与 `sensor_combined`，转换为 FLU。
电机指令按零阶保持对齐，比力线性插值；筛选已解锁 OFFBOARD、有限数值及 [0, 1] 电机指令。
遇到超过 `max(50 ms, 5 × 中位采样间隔)` 的间隙分段，只保留样本数最多的一段，至少 400 个样本。
离线不按五类实验分别切分，也不检查在线计划是否全部完成。

必须事先提供质量、四旋翼 FLU 位置、电机顺序、偏航反扭矩符号和 Gazebo 转速映射。
当前 `mass_kg: 0.0` 是占位值，必须替换；质量不是待辨识参数。

### 数学方法：先推进模型，再刚体转动模型

**① 共同电机动态与二次推力曲线。** 对每个电机的归一化指令 $u_i$，构造内部状态 $q_i$：

$$
\tau_m\dot q_i+q_i=u_i,\qquad
\Omega_i=\operatorname{clip}(\Omega_0+S q_i,0,\Omega_{\max}),\qquad
F_i=k_f\Omega_i^2
$$

代码对 $q_i$ 作零阶保持精确递推，以首个指令初始化。
$\Omega_i$ 是模型推算的机械角速度，不是实测 RPM。
当前配置给定 $S=1360$ rad/s、$\Omega_0=100$ rad/s、$\Omega_{\max}=1360$ rad/s。

以 $F_{\mathrm{obs}}=m f_z$ 为总推力观测，对 $\tau_m\in[0.003,0.15]$ s
取 80 个对数间隔候选。每个候选计算 $a_k=\sum_i\Omega_{i,k}^2$，
再用无截距最小二乘求共同推力系数：

$$
k_f=\frac{\sum_k a_k F_{\mathrm{obs},k}}{\sum_k a_k^2}
$$

选取总推力 RMSE 最小的候选；拒绝非正推力系数或总推力激励不足的数据。
这里辨识的是共同一阶时间常数，不另拟合纯时延，也不分别估计四电机或升速/降速参数。

**② 欧拉刚体方程与线性角阻尼。** 用已知旋翼位置 $(x_i,y_i,z_i)$ 和偏航符号 $s_i$ 构造力矩：

$$
M_x=\sum_i y_i F_i,\qquad M_y=-\sum_i x_i F_i,\qquad
 M_z=k_m\sum_i s_i F_i
$$

假设整机惯量和角阻尼均为对角形式，满足：

$$
\mathbf M=I\dot{\boldsymbol\omega}
 +\boldsymbol\omega\times(I\boldsymbol\omega)+D\boldsymbol\omega,
 \qquad I=\operatorname{diag}(I_{xx},I_{yy},I_{zz})
$$

为避免角速度求导与力矩滤波不一致，将方程两侧在相同约 80 ms 窗口积分：角速度差对应积分角加速度，力矩、角速度与交叉乘积均用梯形积分。将三轴方程堆叠成 $A\theta=b$，同时拟合七个参数：

$$
\theta=[I_{xx},I_{yy},I_{zz},k_m,D_x,D_y,D_z]^T
$$

`rotation_fit_method: torque_residual` 保留原有列归一化、非负力矩残差最小二乘作为对照。当前配置改用 `angular_acceleration`，将每轴方程残差除以对应惯量后拟合，以避免同时缩小偏航惯量、力矩系数和阻尼降低目标值。通过三个正的惯性二阶矩参数化惯量，保证三角不等式；多初值求解，检查核心参数边界。
要求矩阵满秩、条件数 ≤ $10^5$，惯量严格为正且每项不大于另两项之和。
耦合激励使 $\omega_y\omega_z$ 等交叉项参与拟合，但不代表辨识完整非对角惯量矩阵。

### 输出、验证与边界

| 报告字段                                                 | 内容                                                            |
| -------------------------------------------------------- | --------------------------------------------------------------- |
| `propulsion.common_motor_time_constant_s`              | 共同等效电机时间常数；不能冒充独立升速/降速常数 |
| `propulsion.gazebo_motor_constant_n_per_rad_s_squared` | $k_f$，单位 N/(rad/s)²                                       |
| `rotation.aggregate_inertia_diagonal_kg_m2`            | 整机聚合对角惯量 `[Ixx, Iyy, Izz]`，kg·m²                   |
| `rotation.gazebo_moment_constant_m`                    | $k_m$，偏航反扭矩/推力比，单位 m；不是相对于转速平方的系数    |
| `rotation.angular_damping_nm_per_rad_s`                | 三轴线性角阻尼，N·m/(rad/s)                                    |
| `rotation.condition_number`                            | 归一化训练回归矩阵条件数                                        |
| `validation`、`passed`                               | 独立验证误差、阈值与整体通过标志                                |

固定训练参数，用另一份独立 ULog 验证：总推力 RMSE 必须 < $0.12mg$，
三轴“力矩方程残差 / 对应惯量”的 RMSE 均必须 < 3.0 rad/s²，且电机时间常数未落在搜索端点。
角加速度阈值可通过配置键 `max_angular_accel_rmse_rad_s2` 覆盖。
这是基于实测状态的动力学残差验证，不是完整轨迹的自由仿真预测。

模型假设低速飞行、相同且竖直的旋翼、已知转速映射和对角聚合惯量。
不估计质量、转速映射、完整惯量矩阵、平动阻力或各电机差异。
SDF 若有其他带质量的链接，聚合惯量不能直接覆盖 `base_link` 惯量。
报告会保留已知配置、日志路径和 SHA-256；输出文件存在不代表 `passed=true`。

## 3. 构建与运行

```bash
cd /home/holy/InterceptionRL/deployment
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
conda activate holydrone_autotune
python -m pip install numpy scipy pyyaml pyulog matplotlib
```

两种激励均需人工起飞悬停、解锁并进入 OFFBOARD 后由状态机触发；退出 OFFBOARD 中止，
正常完成后请求 POSCTL。恢复阶段通过高度、姿态及水平速度反馈回中。

包内 `_scripts` 提供以下入口（二阶入口见文末）（以下从仓库根目录运行）：

```bash
# 激励采集：默认 simulation；实飞设置 MODE=real_flight。
./deployment/src/holydrone_identification/_scripts/brct1order_excite.sh
./deployment/src/holydrone_identification/_scripts/gazebophysics_excite.sh
# 仅停止本工作区辨识会话。
./deployment/src/holydrone_identification/_scripts/kill.sh

# 离线辨识：默认使用 holydrone_autotune Conda 环境。
./deployment/src/holydrone_identification/_scripts/brct1order_identification.sh \
  --train /data/brct_train.ulg --validation /data/brct_validation.ulg --output /data/brct_report.yaml
./deployment/src/holydrone_identification/_scripts/gazebophysics_identification.sh \
  --train /data/physics_train.ulg --validation /data/physics_validation.ulg \
  --config /data/physics_fit_config.yaml --output /data/physics_report.yaml
```

激励入口支持 `CONFIG`、`OUTPUT`、`MODE` 环境变量及追加的 roslaunch 参数；仿真运行目录为
`${OUTPUT}_px4`，物理激励额外记录 `${OUTPUT}_motor_speed.csv`。每次采集使用新前缀。
任一任务退出或主脚本收到 Ctrl+C/TERM 后清理同次会话。实飞 ULog 保存在飞控 SD 卡。
离线入口支持 `TRAIN`、`VALIDATION`、`OUTPUT`、`CONDA_EXE`、`CONDA_ENV` 环境变量，
命令行参数优先；物理入口还支持 `CONFIG` 和原处理程序的转速/ESC 参数。输出路径必须不存在。

BRCT 采集与离线处理：

```bash
roslaunch holydrone_identification real.launch
# 仿真入口：roslaunch holydrone_identification sim.launch output:=/data/id_train
rosrun holydrone_identification brct1order_process.py \
  --train /data/brct_train.ulg --validation /data/brct_validation.ulg \
  --output /data/brct1order.yaml
```

物理辨识：先填写 `config/gazebo_physics.yaml` 的实测质量，核对旋翼几何、电机顺序和转速映射。

```bash
roslaunch holydrone_identification gazebo_physics_real.launch
rosrun holydrone_identification gazebo_physics_process.py \
  --train /data/physics_train.ulg --validation /data/physics_validation.ulg \
  --config $(rospack find holydrone_identification)/config/gazebo_physics.yaml \
  --output /data/gazebo_physics.yaml
```

训练和验证应来自不同航次；脚本拒绝复用相同数据，并拒绝覆盖已有输出文件。

## 4. Lite SITL 实测与复现（2026-11-03）

本次测试数据在 `deployment/data/identification/validation_20261103`，详见该目录 `REPORT.md`：BRCT 四通道独立验证通过；物理激励完成，但物理参数整体 **未通过**，偏航惯量及力矩系数严重失真，禁止回写 SDF。
不满足惯量三角不等式时现在保留候选参数和 `physical_consistency` 诊断，同时输出 `passed: false`；低残差不能覆盖物理一致性失败。

仿真测试入口（会自动起飞、执行激励、降落；仅用于本机 SITL）：

```bash
source deployment/devel/setup.bash
/usr/bin/python3 deployment/src/holydrone_identification/test/sitl_sequence.py \
  --kind gazebo_physics --px4 /home/holy/PX4-Autopilot --output /tmp/new-physics-flight
```

该入口在 PX4 启动前设置 `PX4_PARAM_SDLOG_PROFILE=17`，记录 250 Hz 电机数据，并保存实际 Gazebo link 质量、惯量和位姿。普通 SITL 启动也应在 PX4 启动前设置该环境变量；仅飞行前通过 MAVROS 修改 SDLOG_PROFILE 不会重建已启动 logger 的订阅。

`test/compare_gazebo_truth.py --session-root <测试目录> --sdf <实际SDF>` 从 `physics_train/gazebo_links.json` 汇总已知质量、质心与整机惯量，生成 `physics_fit_config.yaml`。运行物理分析后再运行比较脚本，生成 `comparison.json/csv/png`。旋翼横向惯量取转动平均，所有 link 使用平行轴定理移到整机质心；不能只拿 base_link 惯量作为真值。插件阻力系数与拟合的常数角阻尼含义不同，不能直接比较数值。

## 5. 实测转速、多时间尺度激励和可选惯量先验

Lite SDF 的 `actual_motor_feedback` 插件在电机力计算之前读取实际关节速度，乘 `rotorVelocitySlowdownSim=10`，发布 `/identification/motor_speed` (`sensor_msgs/JointState`)。`velocity` 是按 motor_0..3 排列的**正机械角速度 rad/s**，不是控制指令、期望转速或电气 RPM。插件只读，不施加力或改变电机动态。

`deployment/src/holydrone_identification/_scripts/gazebophysics_excite.sh` 在仿真模式自动设置高频日志并记录 `${OUTPUT}_motor_speed.csv`，拒绝复用已有会话前缀。也可独立启动：

```bash
rosrun holydrone_identification record_motor_speed.py _output:=/data/new_motor_speed.csv
```

两次独立航次完成后，在 `holydrone_autotune` Conda 环境中运行（从 deployment 目录）：

```bash
python src/holydrone_identification/scripts/gazebo_physics_process.py \
  --train /data/train.ulg --validation /data/validation.ulg \
  --train-rpm /data/train_motor_speed.csv --validation-rpm /data/validation_motor_speed.csv \
  --rpm-ros-minus-px4-s 0 \
  --config /data/physics_fit_config.yaml --output /data/new_physics_report.yaml
```

这里的偏移 0 仅适用于本次已核对的 Gazebo/PX4 lockstep 同时钟；外部采集必须提供实际时钟对齐值。分析拒绝倒退、重复、覆盖不足或超过 20 ms 的转速缺口，不跨丢包补造快速动态。采用实测转速时，直接拟合 `F=kf·ΣΩ²`，不再拟合电机时常，报告该项为 null。

实飞 BDShot 数据入口为 `--esc-indices 0 1 2 3`，替代两个 CSV 和时钟偏移参数。必须核对这四个 ESC 槽位与几何配置的电机顺序。当前 PX4 驱动根据 `MOT_POLE_COUNT` 将电气转速换成 `esc_status.esc[i].esc_rpm` 中的机械 RPM，分析再乘 `2π/60` 转成 rad/s；不要重复除极对数。分析逐电机使用各自 `esc[i].timestamp`，不将整包 ESC 消息的重复数据当成新测量。此入口尚未经过真实 BDShot 飞行验证。

可选扭摆先验通过 `inertia_prior_axes`、`inertia_prior_kg_m2`、`inertia_prior_std_kg_m2`、`inertia_prior_source` 输入。只测 Izz 时 axes 为 `[2]`，均值与标准差各填一个实测值，单位 kg·m²；也可按 `[0,1,2]` 输入三轴。先验是带不确定度的附加残差，不是硬固定参数。必须匹配当前整机配置、质心和机体系；默认不启用，不自动用 SDF 真值填充。

`passed` 仅指独立动态残差和物理约束检查通过，**不等于参数误差已通过**。最新测试在 `deployment/data/identification/rpm_validation_20261103/REPORT.md`：两航次完成，未用惯量先验。推力系数误差约 −0.24%，roll/pitch 惯量约 +9.5%/+7.0%，但 yaw 惯量与力矩系数仍约 +47%/+69%，因此不自动回写。`comparison.json` 单独给出参考误差与 10% 对照门槛。

## 6. 偏航分段正弦扫频

试验配置为 [gazebo_physics_yaw_sweep.yaml](config/gazebo_physics_yaw_sweep.yaml)，普通配置仍保留双脉冲。
每轮把纯偏航脉冲替换成 0.5、1、2、3 Hz 四段正弦，每段 4 周期、幅值 0.25 rad/s；其他轴、推力和耦合组保持原计划。两轮共 46 个激励组，其中 8 个扫频段。段内指令为 `r_cmd=A·sin(2πft)`，按 ROS 时间计算相位，在仿真中不随主机运行速度改变频率。

每段起止指令为零；段内继续高度反馈、倾角补偿，并用 roll/pitch 反馈抑制水平漂移。每段结束后独立回中，满足角速度、姿态、航向、高度、位置和速度条件后再开始下一段。无效频率/周期/幅值拒绝启动；切换配置时清除旧节点私有参数，避免上次的扫频开关残留。

从功能包目录启动人工操作的仿真采集（使用新的 OUTPUT 前缀）：

```bash
CONFIG="$PWD/config/gazebo_physics_yaw_sweep.yaml" \
OUTPUT="$PWD/data/yaw_sweep_train" ./_scripts/gazebophysics_excite.sh
```

隔离 master 的自动 SITL 回归仍使用 `test/sitl_sequence.py`，增加 `--config <扫频YAML>`；该入口会自动起飞、执行实验、降落。离线物理分析继续使用实际转速 CSV、独立训练/验证 ULog 和填好质量/几何的配置，不把偏航设定值当成物理力矩输入。

`test/analyze_yaw_sweep.py --root <测试结果目录>` 另外提取各频段的“实际电机差动推力 → 实测偏航角速度”复频响，丢弃每段首周期，以正弦/余弦加常数和慢趋势项拟合。使用训练频响拟合 `G(s)=a·exp(-d·s)/(s+b)`，在另一航次验证；这里 `a=km/Izz`、`b=Dz/Izz` 仅为有效模型解释，不提供两个绝对参数的独立尺度。输出 `sweep_response.json/png`，与物理参数真值比较分开判断。

本次验证目录：`deployment/data/identification/yaw_sweep_20261103`。是否扩展到其他轴，以该目录报告中的独立验证和参数误差为依据，不仅看频响拟合是否漂亮。

本次结果：两次扫频航次均完成；独立复频响误差约 0.75%，但 Izz/km 真值误差仍为约 +41%/+62%，只比原双脉冲小幅改善。因此保留可选偏航扫频配置，尚未将其他轴默认改为扫频。详见 [扫频验证报告](../../data/identification/yaw_sweep_20261103/REPORT.md)。


## BRCT 二阶试验入口

新增 `src/brct2order_excite.cpp` 定义单轴多时长双脉冲，仍由同一个
`identification_fsm` 执行回中、定高、反馈超时保护和退出模式请求。
`config/brct2order.yaml` 保持一阶的幅值，使用 0.20/0.35/0.50 s 三种平台时长，
每种时长依次覆盖 roll/pitch/yaw/thrust，重复 2 轮：共 24 组、48 个脉冲。
需要人工起飞悬停并进入 OFFBOARD，不自动起飞。该新计划尚未完成实际 SITL/实飞试验。

从仓库根目录执行（修改后先在 deployment 下 `catkin_make`）：

```bash
# 采集两个独立航次；实飞在命令前增加 MODE=real_flight。
OUTPUT="$PWD/deployment/data/identification/brct2order/train01" \
  deployment/src/holydrone_identification/_scripts/brct2order_excite.sh
OUTPUT="$PWD/deployment/data/identification/brct2order/validation01" \
  deployment/src/holydrone_identification/_scripts/brct2order_excite.sh
# 停止一阶、二阶或物理辨识采集会话。
deployment/src/holydrone_identification/_scripts/kill.sh

# 使用相同训练/验证日志公平比较；输出必须是新文件。
deployment/src/holydrone_identification/_scripts/brct1order_identification.sh \
  --train /data/train.ulg --validation /data/validation.ulg --output /data/order1.yaml
deployment/src/holydrone_identification/_scripts/brct2order_identification.sh \
  --train /data/train.ulg --validation /data/validation.ulg --output /data/order2.yaml
```

离线入口 `scripts/brct2order_process.py` 复用一阶 ULog 脉冲提取器，
`src/response2.py` 拟合 `y''+2ζωn y'+ωn²y=ωn²[Ku(t−d)+b]`。
输出自然频率（rad/s 与 Hz）、阻尼比、时延、增益、偏置和独立验证指标。
支持欠阻尼、临界阻尼与过阻尼；每组用首个实测输出初始化，初始导数固定为零。
自然频率、阻尼比等触边时不判为通过；阈值沿用一阶的 NRMSE 和绝对 RMSE 门槛。
自然频率上界受观测采样率约束。参数描述局部闭环响应，不是 Gazebo 电机物理参数。
旧一阶或物理激励日志中完整的单轴双脉冲也可用于二阶拟合；不要把同一日志同时作为训练和验证。

## 7. 独立参数先验文件

填写 [parameter_priors.yaml](config/parameter_priors.yaml)。默认 `enabled: false`，测量值为 null，不会自动使用示例值。默认 `inertia_prior_axes: [2]` 只对应 Izz；需要三轴时改为 `[0, 1, 2]` 并依次填写三项均值、三项 1σ 不确定度（均为 kg·m²）。填写测量来源和机体配置后再启用。

从功能包目录执行现有分析脚本，在原有参数中增加：

```bash
./_scripts/gazebophysics_identification.sh \
  --config /data/physics_fit_config.yaml \
  --priors config/parameter_priors.yaml \
  --train /data/train.ulg --validation /data/validation.ulg \
  --train-rpm /data/train_motor_speed.csv --validation-rpm /data/validation_motor_speed.csv \
  --rpm-ros-minus-px4-s 0 --output /data/new_prior_report.yaml
```

时钟偏移 0 仅用于已核对的本机 lockstep 仿真；实飞沿用此前的 ESC 输入和时间对齐方式。主配置仍需填写已知质量、几何并选择 `rotation_fit_method: angular_acceleration`。

`--priors` 显式指定的文件优先于主配置内的旧惯量先验：启用时替换，禁用时移除旧先验，便于做有/无先验对照；不传该参数则保持原行为。当前独立文件仅支持惯量先验，不接受尚未实现的其他参数先验。启用但缺值、轴重复或不确定度非正时拒绝分析。报告中 `parameter_prior_file` 保存路径、SHA-256 和开关，`rotation_fit` 保存实际使用的均值、不确定度、轴和来源。
