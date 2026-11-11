# holydrone_identification

ROS 1 辨识包，提供 BRCT 一阶/二阶/三阶闭环响应辨识和 Gazebo 物理参数辨识；飞行器、Gazebo、PX4 和 MAVROS 启动资源位于 `holydrone` 包。

| 流程            | 辨识对象                                  | 主要输出                                                   |
| --------------- | ----------------------------------------- | ---------------------------------------------------------- |
| BRCT 一阶响应   | 角速度/总推力指令到实测响应的局部等效动态 | 四通道时间常数、纯时延、增益、偏置，以及悬停推力           |
| BRCT 二阶响应 | 同样的指令/响应数据，使用二阶动态模型 | 四通道自然频率、阻尼比、纯时延、增益、偏置 |
| BRCT 三阶响应 | 一阶惯性串联二阶环节的局部闭环动态 | 四通道惯性时间常数、自然频率、阻尼比、纯时延、增益、偏置 |
| Gazebo 物理辨识 | 四电机指令到推力和刚体转动的物理模型      | 共同电机时间常数、推力系数、对角惯量、偏航力矩系数、角阻尼 |

BRCT 一阶/二阶/三阶默认使用同一次 excite 的一份 ULog，按完整脉冲组划分训练和验证；物理辨识仍使用独立训练、验证 ULog。均不自动修改 PX4 参数或 Gazebo SDF。
采集数据、PX4 运行文件和辨识报告统一放在本包 `data/`，与 `holydrone_autotune` 一致；
实验产物由 Git 忽略，仅保留 `data/.gitkeep`。历史运行日志及训练快照中的旧路径按
`deployment/data/identification/` → `deployment/src/holydrone_identification/data/` 对应，原始日志不改写。
`holydrone_autotune` 的角速度环候选参数试验是另一套流程。

## 1. BRCT 一阶响应辨识

实现：[激励节点](src/brct1order_excite.cpp) → [日志处理](scripts/brct1order_process.py) → [拟合与验证](scripts/brct1order_regression.py)。

### 激励

按 `roll → pitch → yaw → thrust` 依次激励，每次只改变一个通道。
当前 [brct1order.yaml](config/brct1order.yaml)：roll/pitch 为 ±0.20 rad/s，yaw 为 ±0.25 rad/s，推力为每组基准 ±0.015。
统一 [identification_fsm.cpp](src/identification_fsm.cpp) 以 200 Hz 调度，供一阶、二阶、三阶、物理辨识和偏航扫频共用：人工起飞悬停 → 进入 OFFBOARD 捕获位置/航向 → PX4 位置回中并采集悬停推力 → 稳定条件满足后直接启动单个脉冲 → 直接返回 PX4 位置控制、回中 → 下一个脉冲。一阶每个正负平台 0.35 s，重复 3 轮，共 12 组、24 个脉冲。新日志按独立平台提取和划分训练/验证，不把制动段拼接成激励。

回中门槛：高度误差 ≤0.15 m、XY 偏移 ≤0.75 m、航向误差 ≤5°、水平速度 ≤0.3 m/s、倾角 ≤5°、滤波后三轴角速度合量 ≤5°/s，连续新鲜反馈满足 0.3 s 后才继续。稳定窗口和推力采样均在 PX4 位置控制期间完成；满足条件后直接切换到激励指令，不再插入零角速度交接或脉冲后缓冲。推力脉冲以刚采集的悬停推力均值为基准。回中和推力采样不设总等待时限，也不定时重置稳定计时和推力采样；保持 OFFBOARD 时持续等待。退出 OFFBOARD、主动停止或反馈失效等异常仍停止序列，完成全部脉冲后不自动重复。

判稳使用 [AngularRateFilter](include/holydrone_identification/settling_gate.hpp)：按 IMU 源时间戳间隔 $\Delta t$ 对每个角速度分量计算
$\bar{\boldsymbol\omega}_k=e^{-\Delta t/\tau_f}\bar{\boldsymbol\omega}_{k-1}+(1-e^{-\Delta t/\tau_f})\boldsymbol\omega_k$，
再比较 $\|\bar{\boldsymbol\omega}_k\|$。`recovery/angular_rate_filter_tau_s` 默认 $\tau_f=0.05$ s，必须为正且不超过稳定窗口。
这样可避免高频正负波动的瞬时合量不断清空稳定计时；持续转动仍会阻止激励。首样本、反馈间隔超过 `protection/angular_feedback_timeout_s`（默认 0.1 s）、时间戳倒退或拒收后的首样本均从当前测量重新初始化。此滤波仅用于在线判稳，ULog 及离线辨识输入输出保持原始数据。

每次恢复位置控制后隔离 0.65 s，再采集最近 0.5 s 的 FCU 推力，要求至少 3 个样本覆盖 0.2 s，取平均值并重置高度积分。角速度激励使用高度闭环和倾角补偿；推力双脉冲保留冻结基准及饱和检查。偏航扫频仍在激励期间对 roll/pitch 做姿态和水平漂移反馈，组间回中交给 PX4。

保留反馈新鲜度与调度保护：IMU 的 ROS 年龄 ≤0.1 s、接收墙钟年龄 <0.5 s，位姿/速度接收年龄 <0.5 s，FCU 状态 <2 s；脉冲调度超时 0.1 s 或反馈失效会中止。旧的大倾角、高度偏差、总距离、垂直速度硬中止项随 Autotune 一并移除。反馈失效时停止发布设定值，由 PX4 自身保护接管。

完成、超时或人工调用 `stop` 后保持 OFFBOARD 位置控制，不再自动请求 POSCTL；人工退出模式立即停止激励。不自动解锁、起飞或降落。`finished=true` / `sequence_complete=true` 表示最后一组已回中，不表示已降落。旧 `exit_mode` launch 参数已移除。

### 数据与数学方法

辨识使用的不是“正脉冲的一个终值”和“负脉冲的一个终值”，而是整组双脉冲期间的
指令、时间戳和实测响应。对于同一通道，代码用一套参数同时解释所有训练组的完整时间序列。
负脉冲开始时允许实际角速度仍为正值，模型从此前的预测状态继续计算。

#### 1. 实际拟合的输入和输出

[ulog_to_csv()](scripts/brct1order_process.py) 读取 `vehicle_rates_setpoint`、
`vehicle_angular_velocity`、`sensor_combined` 和 `vehicle_control_mode`，保留已解锁 OFFBOARD
且各源流共同覆盖的数据。ULog 的机体系 FRD 转为 FLU：roll 不变，pitch/yaw 取反；
`thrust_body[2]` 和加速度计 Z 分量取反。

| 通道 | 输入 $u(t)$ / CSV 字段 | 输出 $y(t)$ / CSV 字段 | 模型含义 |
| --- | --- | --- | --- |
| roll | `cmd_wx`，rad/s | `wx`，rad/s | roll 角速度目标到实际角速度 |
| pitch | `cmd_wy`，rad/s | `wy`，rad/s | pitch 角速度目标到实际角速度 |
| yaw | `cmd_wz`，rad/s | `wz`，rad/s | yaw 角速度目标到实际角速度 |
| thrust | `cmd_thrust`，归一化绝对推力 | `fz`，m/s² | 总推力目标到机体 Z 比力 |

角速度指令来自飞控日志里的实际 setpoint，不用 YAML 中的理想幅值代替。
指令按零阶保持对齐到角速度时间戳，即采样间继续使用最近一个指令值；比力线性插值到相同时间戳。
$f_z$ 在水平静态悬停时约为 9.81 m/s²，不是扣除重力后的世界系竖直加速度。

前三个通道包含当前 PX4 角速度闭环的效果，识别的是局部等效响应；不是直接由角速度指令求惯量或电机常数。
推力通道拟合绝对推力值，不预先减去悬停基准，因而报告中的增益和偏置可用于计算 `hover_thrust`。

#### 2. 脉冲输入与历史连续双脉冲模型

当前在线流程在正负平台之间位置制动。带 `offboard_control_mode` 的新日志会把成对正负平台分别提取为独立片段，用各片段首个实测值初始化；不将中间的位置控制响应用于辨识。以下连续双脉冲推导用于解释旧日志，不能当作新版在线时序。

令一组正脉冲从 $t=0$ 开始，平台时长为 $T$，幅值为 $A$，基准输入为 $u_0$。
理想双脉冲可以写成：

$$
u(t)=u_0+
\begin{cases}
0,&t<0,\\
+A,&0\le t<T,\\
-A,&T\le t<2T,\\
0,&t\ge 2T.
\end{cases}
$$

也可以用单位阶跃 $H(t)$ 表示：

$$
u(t)=u_0+A\left[H(t)-2H(t-T)+H(t-2T)\right].
$$

三次输入变化分别是 $+A$、$-2A$、$+A$。中间从 $+A$ 直接切到 $-A$，
没有“等待实测角速度归零”的组内阶段。当前一阶配置的 roll 为 $A=0.20$ rad/s、$T=0.35$ s、
$u_0=0$；推力通道的 $u_0$ 是本组冻结的推力基准、$A=0.015$。

这个分段表达用于解释波形。实际回归始终使用日志中的 $u(t)$：组后会从角速度控制切回 PX4
位置控制，因此日志前后沿不保证严格为零；推力基准也可能因组次而不同。

#### 3. 一阶模型怎样跨过正负切换点

[brct1order_regression.py](scripts/brct1order_regression.py) 对每个通道拟合：

$$
\tau\dot y(t)+y(t)=K u(t-d)+b,
\qquad \theta_1=(\tau,d,K,b).
$$

$\tau$ 为时间常数，决定变化快慢；$d$ 为纯时延，决定输入变化何时开始作用；
$K$ 为稳态增益；$b$ 为输出偏置。每个通道有自己的四个参数，不要求四通道相同。

对于一个延迟输入保持为常值 $\bar u$ 的小区间 $[t_a,t_b]$，解为：

$$
\hat y(t_b)=\alpha\hat y(t_a)+(1-\alpha)(K\bar u+b),
\qquad \alpha=\exp\left[-\frac{t_b-t_a}{\tau}\right].
$$

`predict()` 只在每组最开始设置 $\hat y(t_0)=y(t_0)$，以后一直递推。
如果延迟后的指令跳变 $t_j+d$ 落在两个观测时刻之间，代码先在该跳变点拆开积分区间，
分别使用跳变前后的输入，避免将纯时延粗略取整到某个采样点。

因此，正负切换时满足：

$$
\hat y((T+d)^+)=\hat y((T+d)^-).
$$

输入目标从 $u_0+A$ 变成 $u_0-A$，输出本身连续；后续只是朝新的稳态值
$K(u_0-A)+b$ 演化。这里没有把预测输出重置为零，也没有把实测负脉冲首值再次喂给模型。

在“脉冲前处于基准稳态”的理想条件下，令 $y_b=Ku_0+b$，并定义一阶单位阶跃响应
$S_\tau(x)=(1-e^{-x/\tau})H(x)$，可将整组输出写成：

$$
\hat y(t)=y_b+KA\left[
S_\tau(t-d)-2S_\tau(t-T-d)+S_\tau(t-2T-d)
\right].
$$

这是同一递推模型的直观表达，不是另一套拟合算法。代码允许组首输出偏离稳态，
通过首个实测值初始化处理，而不强行假设 $y(t_0)=y_b$。

**数值示意，不是辨识结果。** 取配置中的 roll 幅值 $A=0.20$ rad/s、$T=0.35$ s；
另外假设 $\tau=0.05$ s、$d=0$、$K=1$、$b=0$、$y(0)=0$，则：

$$
\hat y(T)=A(1-e^{-T/\tau})\approx0.1998\ \mathrm{rad/s},
$$

$$
\hat y(2T)=-A+\big[\hat y(T)+A\big]e^{-T/\tau}
\approx-0.1996\ \mathrm{rad/s}.
$$

负脉冲从约 $+0.1998$ rad/s 开始下降，会自然经过零再接近负目标。
辨识利用这段完整过渡，不需要等实测回零才开始负脉冲。

#### 4. 二阶模型如何使用同一组连续输入

[brct2order_regression.py](scripts/brct2order_regression.py) 使用相同输入、输出和数据划分，
将模型换成：

$$
\ddot y+2\zeta\omega_n\dot y+\omega_n^2y
=\omega_n^2\left[K u(t-d)+b\right],
\qquad \theta_2=(\omega_n,\zeta,d,K,b).
$$

$\omega_n$ 为自然频率，单位 rad/s，报告中同时给出 $f_n=\omega_n/(2\pi)$ Hz；
$\zeta$ 为阻尼比。欠阻尼时模型允许超调和衰减振荡，临界/过阻尼时也使用相应的解析解。
一阶模型只有一个动态状态，二阶模型还包含输出变化率，因此能描述不同的瞬态形状。

代码每组设置 $\hat y(t_0)=y(t_0)$、$\dot{\hat y}(t_0)=0$；
**初始导数为零是当前实现的假设，不是从日志中估计的参数。** 若组首仍有明显动态，
这一假设可能影响拟合。正负脉冲切换时不会重新设置输出或导数。

令 $h(x)$ 为初始输出为 1、初始导数为 0 的齐次响应，单位阶跃响应为
$S_2(x)=[1-h(x)]H(x)$。欠阻尼 $0<\zeta<1$ 时：

$$
h(x)=e^{-\zeta\omega_n x}
\left[\cos(\omega_d x)+\frac{\zeta\omega_n}{\omega_d}\sin(\omega_d x)\right],
\qquad \omega_d=\omega_n\sqrt{1-\zeta^2}.
$$

临界阻尼时 $h(x)=e^{-\omega_n x}(1+\omega_n x)$；过阻尼时，令
$r_1=-\omega_n(\zeta-\sqrt{\zeta^2-1})$、
$r_2=-\omega_n(\zeta+\sqrt{\zeta^2-1})$，则
$h(x)=(r_2e^{r_1x}-r_1e^{r_2x})/(r_2-r_1)$。

对于日志中任意一串阶跃，令 $\Delta u_j=u(t_j)-u(t_j^-)$、$y_b=Ku(t_0)+b$，
`predict()` 用阶跃叠加计算整组预测：

$$
\hat y(t)=y_b+[y(t_0)-y_b]h(t-t_0)
+K\sum_j\Delta u_j S_2(t-t_j-d).
$$

理想正负双脉冲的增量仍是 $A,-2A,A$；代码则使用全部实际输入跳变。
二阶计划的 0.20/0.35/0.50 s 平台提供不同时间尺度，同一组参数必须同时解释这些响应，
而不是为每种平台单独拟合后取平均。增加模型阶数或脉冲时长种类不保证参数一定可靠，仍以留出误差和边界检查判断。

#### 5. 参数具体怎样求出来

对于某通道的第 $g$ 个训练组，给定候选参数 $\theta$，先用上面的模型生成整组预测，
再计算每个采样点的残差：

$$
r_{g,k}(\theta)=\hat y_g(t_{g,k};\theta)-y_g(t_{g,k}).
$$

把所有训练组的残差拼成一个向量，交给 `scipy.optimize.least_squares`。
当前使用 `soft_l1` 鲁棒损失：

$$
\hat\theta=\operatorname*{argmin}_{\theta\in\mathcal B}
\sum_{g,k}c^2\rho\left(\frac{r_{g,k}(\theta)^2}{c^2}\right),
\quad \rho(z)=2(\sqrt{1+z}-1),\quad c=0.1.
$$

$\mathcal B$ 是参数边界。小残差近似平方损失，大残差的影响相对减弱，但不会自动消除坏数据。
$c$ 在三个角速度通道按 rad/s、推力输出通道按 m/s² 解释；代码没有先统一标准化输出。
一阶首个样本的预测误差恒为零，仍在拟合残差中；二阶拟合显式去掉每组首个样本。
两种模型评分时都排除每组首个样本。

| 实现 | 优化初值和边界 | 候选选择 |
| --- | --- | --- |
| 一阶 | 初始 $\tau=0.04$ s，三个 $d=0.005/0.03/0.08$ s；初始 $K$ 为角速度 1、推力 30，$b=0$；$\tau\in[0.003,0.5]$ s、$d\in[0,0.15]$ s、$K\in[0.01,100]$、$b\in[-20,20]$ | 三次优化后按训练残差 MSE 选优，选中结果未收敛时报错 |
| 二阶 | 三组 $(\omega_n,\zeta,d)$ 初值为 $(15,0.5,0.01)$、$(40,0.5,0.02)$、$(80,1.2,0.04)$；$\omega_n$ 初值按上界截低，初始 $K,b$ 同上；$f_n\in[1,\min(80,0.45/\Delta t_{\max})]$ Hz、$\zeta\in[0.1,2]$，$d,K,b$ 边界同上 | 只保留收敛结果，再按训练残差 MSE 选优 |

表中 $\Delta t_{\max}$ 是训练组中最大的“组内采样间隔中位数”，不是发布定时器周期。
训练样本直接拼接，没有按组等权或对噪声协方差加权，所以样本较多的组贡献较多残差。
这也是不能仅凭两个平台终值分别求四个或五个参数的原因：时间常数、时延或阻尼必须由瞬态形状约束。

#### 6. 哪些片段会进入拟合，怎样留出验证

`pulse_groups()` 在实际指令中寻找单通道、近似等幅、符号相反的两个平台；
其他角速度轴须接近零（绝对值 <0.005 rad/s），角速度实验允许定高推力变化。
正负平台间隔必须 ≤0.08 s，平台采样跨度至少 0.12 s，两平台时长较小值至少为较大值的 70%，
整对跨度至少 0.4 s。推力脉冲方向相对于日志悬停推力中位数判断，基准漂移过大时可能丢组。
这是不带 `offboard_control_mode` 的旧日志兼容路径。新日志使用该话题排除位置/姿态控制平台，允许匹配正负平台间最多 15 s 的制动间隔，并输出两个独立片段；片段扩展窗口限制在连续角速度控制区间内。

提取窗口在整对脉冲前后各扩展至多 0.25 s，与邻组之间在中点截断以避免共享采样。
窗口可能包含交接或位置回中产生的角速度 setpoint；当前没有按控制类型进一步剔除这些点，
并始终以实际日志输入做预测，不能把整个窗口假定为理想的 $0,+A,-A,0$。
`segments()` 还要求至少 20 个连续有效样本、有限数值、最大采样间隔 ≤0.05 s 且不超过组内中位数的 3 倍；
输入跨度至少为角速度 0.08 rad/s、推力 0.008。它不拼接相隔很远的组，也不跨无效反馈插值。
离线不读取 ROS 阶段状态，因而识别到双脉冲不等于已核验在线计划全部完成。

单日志模式先按“通道 + 平台时长”分层；时长由整对跨度和采样间隔估计，再按 0.05 s 档位归类。
每层有 $n\ge2$ 组时，默认 $q=1/3$，留出数为：

$$
n_{\mathrm{val}}=\min\left(n-1,\max\left(1,\lceil nq\rceil\right)\right).
$$

按时间取最后 $n_{\mathrm{val}}$ 个完整组做验证，其余训练。不随机拆组内采样点，也不把正负两半分到不同集合。
旧日志连续双脉冲按组划分；新日志默认一阶每通道 6 个独立平台，前 4 个训练、后 2 个验证。二阶按通道和平台时长分层后使用同样比例。
拟合只用训练组，参数固定后计算验证组预测误差。验证组也仅用首个实测输出初始化，不在后续采样点校正预测，
不是逐步把实测输出喂回模型的一步预测。该流程属于同航次留出验证，不表示已验证不同载荷、风扰或控制参数。

### 输出与判定

每通道输出 `tau_s`、`delay_s`、`gain`、`bias`，以及训练/验证误差、边界标志和
`validated_input_range`（训练、验证输入区间交集）。角速度增益无量纲，偏置为 rad/s；
推力增益单位为 m/s² / 单位归一化推力，偏置为 m/s²。
由推力稳态模型推算 `hover_thrust`：

$$
u_{\mathrm{hover}}=(9.81-b_T)/K_T
$$

固定训练参数，在同航次留出的完整脉冲组上验证。NRMSE 为 RMSE 除以实测输出的 P95−P5 跨度：

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
当前 [gazebo_physics.yaml](config/gazebo_physics.yaml) 使用 roll/pitch ±0.30 rad/s、yaw ±0.25 rad/s、推力增量 ±0.025。每组由两个独立正负脉冲组成，各脉冲之间位置制动，`pulse_durations_s: [0.20, 0.35, 0.50]` 提供不同时间尺度。若按一对正负平台定义一个方波周期，这些是平台时长设置，脉冲间制动会打断周期，不能视为连续方波频率。
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
cd /home/holy/HoverRL/deployment
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
conda activate holydrone_autotune
python -m pip install numpy scipy pyyaml pyulog matplotlib
```

两种激励均需人工起飞悬停、解锁并进入 OFFBOARD 后由状态机触发；退出 OFFBOARD 中止，
正常完成后保持 OFFBOARD 位置。恢复阶段由 PX4 控制位置和航向。

包内 `_scripts` 提供以下入口（二阶入口见文末）（以下从仓库根目录运行）：

```bash
# 激励采集：仿真入口；实飞使用 real_flight/ 下同名脚本，不再设置 MODE。
./deployment/src/holydrone_identification/_scripts/simulation/brct1order_excite.sh
./deployment/src/holydrone_identification/_scripts/simulation/gazebophysics_excite.sh
# 仅停止本工作区辨识会话。
./deployment/src/holydrone_identification/_scripts/simulation/kill.sh

# 离线辨识：默认使用 holydrone_autotune Conda 环境。
./deployment/src/holydrone_identification/_scripts/simulation/brct1order_identification.sh
./deployment/src/holydrone_identification/_scripts/simulation/gazebophysics_identification.sh
```

所有 Shell 入口均为零参数脚本，配置集中在参数区，不读取环境变量覆盖。
激励入口直接启动节点、追加记录 PID 后返回；用同目录 kill.sh 停止对应进程组，仿真与实飞分别记录。
Shell 不做额外检查、终端会话管理、退出清理或日志重定向；节点内部控制规则保持不变。
仿真运行目录为 `${OUTPUT}_px4`，物理激励记录 `${OUTPUT}_motor_speed.csv`，这些是辨识所需的采集数据。
实飞 ULog 保存在飞控 SD 卡，落地 disarm 后由下载节点写到 `${OUTPUT}_px4/log/flight_001/`。
离线入口先填写参数区的 ULOG 或 TRAIN/VALIDATION、OUTPUT、CONDA_ENV 等，再直接运行。
物理辨识额外参数可填写在 EXTRA_ARGS 数组；高级输入方式仍可直接使用 Python 命令行接口。
输出文件要求及报告生成由处理程序负责；BRCT 另保存同前缀的 `_train.csv` 和 `_validation.csv`。
BRCT 一阶和二阶辨识均生成同名 `.png`：四个通道纵向排列，每个通道占满画布宽度，蓝色为拟合数据、橙色为验证数据、黑色虚线为模型预测。各脉冲独立预测并留空分隔，横轴为拼接后的脉冲时间，省略回中等待；每段预测以首个实测输出初始化，二阶模型同时采用零初始导数。

BRCT 采集与离线处理：

```bash
roslaunch holydrone_identification real.launch
# 仿真入口：roslaunch holydrone_identification sim.launch output:=/data/id_train
rosrun holydrone_identification brct1order_process.py \
  --ulog /data/brct_flight.ulg --output /data/brct1order.yaml
```

物理辨识：先填写 `config/gazebo_physics.yaml` 的实测质量，核对旋翼几何、电机顺序和转速映射。

```bash
roslaunch holydrone_identification gazebo_physics_real.launch
rosrun holydrone_identification gazebo_physics_process.py \
  --train /data/physics_train.ulg --validation /data/physics_validation.ulg \
  --config $(rospack find holydrone_identification)/config/gazebo_physics.yaml \
  --output /data/gazebo_physics.yaml
```

BRCT 单日志模式不随机拆采样点，而是按通道及脉冲时长分层，将较晚的完整脉冲组留出验证。默认留出约 1/3，每个层至少保留一组训练和一组验证；每种通道/时长至少需要 2 个完整组。两集合没有共享脉冲或采样。报告明确这是同航次验证。物理辨识仍要求不同航次；所有入口拒绝覆盖已有输出。

## 4. Lite SITL 实测与复现（2026-11-03）

本次测试数据在 `deployment/src/holydrone_identification/data/validation_20261103`，详见该目录 `REPORT.md`：BRCT 四通道独立验证通过；物理激励完成，但物理参数整体 **未通过**，偏航惯量及力矩系数严重失真，禁止回写 SDF。
不满足惯量三角不等式时现在保留候选参数和 `physical_consistency` 诊断，同时输出 `passed: false`；低残差不能覆盖物理一致性失败。

仿真测试入口（会自动起飞、执行激励、降落；仅用于本机 SITL）：

```bash
source deployment/devel/setup.bash
/usr/bin/python3 deployment/src/holydrone_identification/test/sitl_sequence.py \
  --kind gazebo_physics --px4 /home/holy/PX4-Autopilot-holydrone-lite --output /tmp/new-physics-flight
```

该入口在 PX4 启动前设置 `PX4_PARAM_SDLOG_PROFILE=17`，记录 250 Hz 电机数据，并保存实际 Gazebo link 质量、惯量和位姿。普通 SITL 启动也应在 PX4 启动前设置该环境变量；仅飞行前通过 MAVROS 修改 SDLOG_PROFILE 不会重建已启动 logger 的订阅。

`test/compare_gazebo_truth.py --session-root <测试目录> --sdf <实际SDF>` 从 `physics_train/gazebo_links.json` 汇总已知质量、质心与整机惯量，生成 `physics_fit_config.yaml`。运行物理分析后再运行比较脚本，生成 `comparison.json/csv/png`。旋翼横向惯量取转动平均，所有 link 使用平行轴定理移到整机质心；不能只拿 base_link 惯量作为真值。插件阻力系数与拟合的常数角阻尼含义不同，不能直接比较数值。

## 5. 实测转速、多时间尺度激励和可选惯量先验

Lite SDF 的 `actual_motor_feedback` 插件在电机力计算之前读取实际关节速度，乘 `rotorVelocitySlowdownSim=10`，发布 `/identification/motor_speed` (`sensor_msgs/JointState`)。`velocity` 是按 motor_0..3 排列的**正机械角速度 rad/s**，不是控制指令、期望转速或电气 RPM。插件只读，不施加力或改变电机动态。

`deployment/src/holydrone_identification/_scripts/simulation/gazebophysics_excite.sh` 在仿真模式自动设置高频日志并记录 `${OUTPUT}_motor_speed.csv`，拒绝复用已有会话前缀。也可独立启动：

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

`passed` 仅指独立动态残差和物理约束检查通过，**不等于参数误差已通过**。最新测试在 `deployment/src/holydrone_identification/data/rpm_validation_20261103/REPORT.md`：两航次完成，未用惯量先验。推力系数误差约 −0.24%，roll/pitch 惯量约 +9.5%/+7.0%，但 yaw 惯量与力矩系数仍约 +47%/+69%，因此不自动回写。`comparison.json` 单独给出参考误差与 10% 对照门槛。

## 6. 偏航分段正弦扫频

试验配置为 [gazebo_physics_yaw_sweep.yaml](config/gazebo_physics_yaw_sweep.yaml)，普通配置仍保留双脉冲。
每轮把纯偏航脉冲替换成 0.5、1、2、3 Hz 四段正弦，每段 4 周期、幅值 0.25 rad/s；其他轴、推力和耦合组保持原计划。两轮共 46 个激励组，其中 8 个扫频段。段内指令为 `r_cmd=A·sin(2πft)`，按 ROS 时间计算相位，在仿真中不随主机运行速度改变频率。

每段起止指令为零；段内继续高度反馈、倾角补偿，并用 roll/pitch 反馈抑制水平漂移。每段结束后独立回中，满足角速度、姿态、航向、高度、位置和速度条件后再开始下一段。无效频率/周期/幅值拒绝启动；切换配置时清除旧节点私有参数，避免上次的扫频开关残留。

从功能包目录启动人工操作的仿真采集（使用新的 OUTPUT 前缀）：

```bash
# 先在脚本参数区将 CONFIG 设为 gazebo_physics_yaw_sweep.yaml，填写 OUTPUT。
./_scripts/simulation/gazebophysics_excite.sh
```

隔离 master 的自动 SITL 回归仍使用 `test/sitl_sequence.py`，增加 `--config <扫频YAML>`；该入口会自动起飞、执行实验、降落。离线物理分析继续使用实际转速 CSV、独立训练/验证 ULog 和填好质量/几何的配置，不把偏航设定值当成物理力矩输入。

`test/analyze_yaw_sweep.py --root <测试结果目录>` 另外提取各频段的“实际电机差动推力 → 实测偏航角速度”复频响，丢弃每段首周期，以正弦/余弦加常数和慢趋势项拟合。使用训练频响拟合 `G(s)=a·exp(-d·s)/(s+b)`，在另一航次验证；这里 `a=km/Izz`、`b=Dz/Izz` 仅为有效模型解释，不提供两个绝对参数的独立尺度。输出 `sweep_response.json/png`，与物理参数真值比较分开判断。

本次验证目录：`deployment/src/holydrone_identification/data/yaw_sweep_20261103`。是否扩展到其他轴，以该目录报告中的独立验证和参数误差为依据，不仅看频响拟合是否漂亮。

本次结果：两次扫频航次均完成；独立复频响误差约 0.75%，但 Izz/km 真值误差仍为约 +41%/+62%，只比原双脉冲小幅改善。因此保留可选偏航扫频配置，尚未将其他轴默认改为扫频。详见 [扫频验证报告](data/yaw_sweep_20261103/REPORT.md)。


## BRCT 二阶试验入口

新增 `src/brct2order_excite.cpp` 定义单轴多时长双脉冲，仍由同一个
`identification_fsm` 执行位置回中、推力采样、控制交接、定高和反馈超时保护；完成后保持 OFFBOARD 位置。
`config/brct2order.yaml` 保持一阶的幅值，使用 0.20/0.35/0.50 s 三种平台时长，
每种时长依次覆盖 roll/pitch/yaw/thrust，重复 2 轮：共 24 组、48 个脉冲。
需要人工起飞悬停并进入 OFFBOARD，不自动起飞。该新计划尚未完成实际 SITL/实飞试验。

从仓库根目录执行（修改后先在 deployment 下 `catkin_make`）：

```bash
# 采集一个完整航次；实飞使用 real_flight/ 下同名脚本。
# 先在脚本参数区填写 OUTPUT。
deployment/src/holydrone_identification/_scripts/simulation/brct2order_excite.sh
# 停止一阶、二阶或物理辨识采集会话。
deployment/src/holydrone_identification/_scripts/simulation/kill.sh

# 使用同一份日志、相同划分公平比较；输出必须是新文件。
# 在参数区设置同一 ULOG 和独立 OUTPUT。
deployment/src/holydrone_identification/_scripts/simulation/brct1order_identification.sh
# 在参数区设置同一 ULOG 和独立 OUTPUT。
deployment/src/holydrone_identification/_scripts/simulation/brct2order_identification.sh
```

离线入口 [brct2order_process.py](scripts/brct2order_process.py) 复用一阶的提取和划分流程，
[brct2order_regression.py](scripts/brct2order_regression.py) 拟合二阶模型；连续脉冲、初始条件和优化方法见上文“数据与数学方法”。
输出自然频率（rad/s 与 Hz）、阻尼比、时延、增益、偏置和留出验证指标。
支持欠阻尼、临界阻尼与过阻尼；每组用首个实测输出初始化，初始导数固定为零。
自然频率、阻尼比等触边时不判为通过；阈值沿用一阶的 NRMSE 和绝对 RMSE 门槛。
自然频率上界受观测采样率约束。参数描述局部闭环响应，不是 Gazebo 电机物理参数。
二阶按通道和 0.20/0.35/0.50 s 时长分别留出后续完整脉冲，默认每层两次重复各分一组训练、一组验证。旧日志有足够重复单轴双脉冲时也可直接使用 `--ulog`。仍兼容 `--train` 和 `--validation` 两航次模式；不将同一组数据重复用于训练和验证。

## BRCT 三阶试验入口

实现：[多时长激励](src/brct3order_excite.cpp) → [单航次处理](scripts/brct3order_process.py) → [三阶回归](scripts/brct3order_regression.py)。
[brct3order.yaml](config/brct3order.yaml) 使用 0.20、0.35、0.50 s 三种平台时长；每种时长依次执行 roll、pitch、yaw、thrust 的正、负脉冲，重复 3 轮，共 36 对、72 个独立脉冲。
幅值仍为 roll/pitch ±0.20 rad/s、yaw ±0.25 rad/s、推力基准 ±0.015。
本包内沿用二阶的多时长计划生成方法，模型阶次由离线回归决定；不是通过三个脉冲定义“三阶”。
每个正、负脉冲之后均直接返回位置控制，回中并重新采集推力；稳定后直接启动下一脉冲。整段用时取决于每次恢复所需时间。
人工起飞并进入 OFFBOARD 后自动执行，完成后保持 OFFBOARD 位置控制。

### 三阶模型及预测

对四个通道分别拟合一阶惯性与二阶环节串联、带纯时延的稳定全极点模型：

$$
G(s)=\frac{K\omega_n^2 e^{-ds}}
{(\tau s+1)(s^2+2\zeta\omega_n s+\omega_n^2)},\qquad y=G[u]+b.
$$

这里 $\tau$ 单位 s，$\omega_n$ 单位 rad/s，$\zeta$ 无量纲，$d$ 单位 s；$K$、$b$ 分别为稳态增益和输出偏置。
$G$ 描述零初始条件下的输入响应；实际片段按下面的实测初始输出初始化。等价微分方程为：

$$
\tau y^{(3)}+(1+2\zeta\omega_n\tau)\ddot y
 +(2\zeta\omega_n+\omega_n^2\tau)\dot y+\omega_n^2 y
 =\omega_n^2\bigl(Ku(t-d)+b\bigr).
$$

每个独立片段取首个实测值 $y_0$，假设初始一阶、二阶导数为零；片段前输入延拓为首个输入 $u_0$。
不跨位置控制段延续状态，也不对实测角速度数值微分三次。
若一个旧日志片段包含连续正负切换，则在该片段内连续预测状态，负脉冲开始时不重新置零。

为计算离散采样上的精确零阶保持响应，使用状态 $x=[z,y,\dot y/\omega_n]^T$，其中 $z$ 为一阶惯性环节输出：

$$
A=\begin{bmatrix}-1/\tau&0&0\\0&0&\omega_n\\
\omega_n&-\omega_n&-2\zeta\omega_n\end{bmatrix},\quad
h(t)=[0,1,0]e^{At}[1,1,0]^T,\quad S(t)=1-h(t)\;(t>0).
$$

令 $S(t)=0$ 对 $t\le0$，$\Delta u_j=u_j-u_{j-1}$，输入跳变时刻为 $t_j$，则：

$$
\hat y(t)=Ku_0+b+(y_0-Ku_0-b)h(t-t_0)
 +K\sum_j\Delta u_j S(t-t_j-d).
$$

这样直接使用日志里的实际时间戳、正负幅值和所有输入跳变；纯时延不要求为整数采样周期。
一般用特征分解求矩阵指数响应，临界阻尼或重合极点时改用矩阵指数，避免重根处的除零和数值失稳。

### 拟合、留出与适用范围

对所有训练片段联合优化 $[\tau,\omega_n,\zeta,d,K,b]$，采用四组初值、`soft_l1` 鲁棒最小二乘；只在训练集选择收敛解，验证集不参与参数选择。
范围为 $\tau\in[0.003,0.5]$ s、$\zeta\in[0.1,2]$、$d\in[0,0.15]$ s、$K\in[0.01,100]$、$b\in[-20,20]$；
自然频率为 1 Hz 至 $\min(80,0.45f_s)$ Hz，$f_s$ 取训练片段中最慢的中位采样率。
报告另行检查三个极点的模长除以 $2\pi$ 均小于训练及验证采样率的 0.45 倍；这只是分辨率筛查，不能证明参数唯一可辨识。

一次 ULog 按通道和平台时长分层留出后续完整脉冲，默认前两轮训练、末轮验证。完整默认采集得到 48 个训练片段、24 个验证片段；每个集合都有正、负脉冲。
共享的提取器仍会拒绝不完整、反馈失效、采样缺口或幅值不足的数据，因此实际片段数可能减少；某层不足两段时拒绝划分。
这是同航次留出验证；需要检查跨航次稳定性时，使用 `--train` / `--validation` 两份日志。

通过条件沿用一、二阶的输入覆盖、输出幅度、NRMSE ≤0.20，以及角速度 RMSE ≤0.15 rad/s、比力 RMSE ≤1 m/s²，并要求参数未贴近搜索边界、采样分辨率达标。
零时延是允许的边界。悬停推力仍为 $(9.81-b_{thrust})/K_{thrust}$，必须在训练、验证共同覆盖的推力范围内。
输出 YAML 包含六个参数、三阶分母系数（按 $s^3,s^2,s,1$ 顺序）、预测误差、通过标志、数据划分和源文件哈希，另生成训练/验证 CSV 与四通道预测 PNG。

三阶允许更丰富的瞬态，但参数更多，并不保证比二阶更合适。建议用同一日志、同一划分比较留出误差。
过阻尼时三个实极点可有多种等效的一阶/二阶分解，不能仅凭曲线吻合就把 $\tau$ 解释成真实电机时间常数。
当前模型不含零点，三个角速度通道描述当前飞控闭环的局部响应；`passed` 不表示物理参数唯一或已完成实飞验证。

### 使用

从仓库根目录执行；使用前在对应脚本参数区填写路径。实飞/仿真入口分开放置，离线脚本均使用 `holydrone_autotune` Conda 环境：

```bash
# 采集：手动起飞悬停，进入 OFFBOARD 后执行。
deployment/src/holydrone_identification/_scripts/real_flight/brct3order_excite.sh
# 仿真使用 _scripts/simulation/brct3order_excite.sh。
# 实飞落地、上锁后等待 ULog saved，再使用同目录 kill.sh 结束会话。

# 辨识：把脚本 ULOG 改成实际 .ulg 路径，OUTPUT 使用新文件名。
deployment/src/holydrone_identification/_scripts/real_flight/brct3order_identification.sh
# 仿真使用 _scripts/simulation/brct3order_identification.sh。
```

也可在该 Conda 环境中直接指定日志和输出：

```bash
python -I deployment/src/holydrone_identification/scripts/brct3order_process.py \
  --ulog /data/flight.ulg --output /data/third_order_report.yaml
```

已有二阶多时长采集也可以直接用三阶脚本拟合，无需只为切换数学模型重新飞行；只有缺少完整重复脉冲时才需要重新采集。
2026-11-08 已完成默认三阶配置的一次完整 Gazebo/PX4 SITL：72 个脉冲全部执行，激励阶段 188.368 s，随后降落并上锁。三阶拟合误差达标，但 roll/yaw 阻尼比贴近下界，整体判为未通过；同数据二阶对照通过。实际飞行尚未验证。详见 [实验报告](data/brct3order/sitl_20261108_153607/REPORT.md)（HTML、原始日志、CSV 和图表同目录）。
本机验证：编译及隔离目录安装通过，29 项 Python 测试、11 项 C++ 测试和隔离 ROS 状态机回归通过。
状态机回归使用一轮和缩短的平台时长验证三阶完整执行及原有保护；合成数据测试覆盖默认 72 个脉冲的划分、六参数恢复、YAML/CSV/PNG 输出与重合极点预测。

## 7. 独立参数先验文件

填写 [parameter_priors.yaml](config/parameter_priors.yaml)。默认 `enabled: false`，测量值为 null，不会自动使用示例值。默认 `inertia_prior_axes: [2]` 只对应 Izz；需要三轴时改为 `[0, 1, 2]` 并依次填写三项均值、三项 1σ 不确定度（均为 kg·m²）。填写测量来源和机体配置后再启用。

从功能包目录执行现有分析脚本，在原有参数中增加：

```bash
./_scripts/simulation/gazebophysics_identification.sh \
  --config /data/physics_fit_config.yaml \
  --priors config/parameter_priors.yaml \
  --train /data/train.ulg --validation /data/validation.ulg \
  --train-rpm /data/train_motor_speed.csv --validation-rpm /data/validation_motor_speed.csv \
  --rpm-ros-minus-px4-s 0 --output /data/new_prior_report.yaml
```

时钟偏移 0 仅用于已核对的本机 lockstep 仿真；实飞沿用此前的 ESC 输入和时间对齐方式。主配置仍需填写已知质量、几何并选择 `rotation_fit_method: angular_acceleration`。

`--priors` 显式指定的文件优先于主配置内的旧惯量先验：启用时替换，禁用时移除旧先验，便于做有/无先验对照；不传该参数则保持原行为。当前独立文件仅支持惯量先验，不接受尚未实现的其他参数先验。启用但缺值、轴重复或不确定度非正时拒绝分析。报告中 `parameter_prior_file` 保存路径、SHA-256 和开关，`rotation_fit` 保存实际使用的均值、不确定度、轴和来源。

## 实飞日志

辨识启动入口不再启动 ULog 自动下载节点。实验后手动获取飞控 `.ulg` 日志，再在离线辨识脚本中指定日志路径。

### OFFBOARD 进入前诊断

辨识节点不解锁、不自动切换模式。在已解锁的 POSCTL/AUTO.LOITER 且状态、位置、速度和 IMU 反馈有效时，先连续发送位置目标；无需先收到推力或进入 OFFBOARD。人工进入 OFFBOARD 后才锁定原点并开始辨识。

`/identification/status` 的 `command_gate` 表明当前发送条件或阻断原因，`command_hz`/`command_count` 是本节点实际发布的频率与计数，`position_subscribers` 为 ROS 订阅者数量；它们不证明飞控已接收。`imu_age_s`、位置/速度接收年龄及各类 `*_rejected` 计数用于检查时间戳和反馈门控。Position 悬停期间终端每 5 秒打印一次预发送诊断，门控变化日志最多每秒一次。`thrust_ready=false` 在进入 OFFBOARD 前是正常状态，不能据此判定预发送未就绪。

进入 OFFBOARD 后用 `excitation_gate` 查看未开始激励的具体条件，例如 `filtered angular speed`、`height error`、`settling window` 或 `waiting for valid FCU thrust`。`body_rate_rad_s` 保留原始角速度合量，`settled_body_rate_rad_s` 为判稳滤波值，`settled_s` 为连续达标时间，`thrust_sample_count` / `thrust_sample_span_s` 为推力采样数量和覆盖时长。等待期间每 2 秒打印一次上述主要信息；回中超时的 `reason` 同时保留具体阻塞条件。
