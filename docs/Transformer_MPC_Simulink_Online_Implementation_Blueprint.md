# Transformer-MPC Simulink 在线闭环实现蓝图

> **For agentic workers:** 按任务清单逐项执行；每个任务完成后先运行该任务的验收命令，再进入下一任务。

**Goal:** 在 MATLAB/Simulink + CarSim 联合仿真中，落地“Transformer 在线风险推理 + 风险监督 + 横向 MPC + 纵向速度外环”的可复现闭环工程。

**Architecture:** 顶层模型采用固定尺寸 Bus 和显式速率调度。Transformer 以低频节拍推理，Risk Supervisor 负责限幅、平滑、规则回退和参数映射，横向 MPC 以高频节拍求解，纵向速度外环独立输出加速度命令。

**Tech Stack:** MATLAB/Simulink、CarSim-Simulink 联合仿真、Deep Learning Toolbox、Model Predictive Control Toolbox（`mpcActiveSetSolver`）、Simulink Data Dictionary、`matlab.unittest`。

---

## 1. 使用范围与完成定义

本文件是对 `Transformer_MPC_Simulink_Online_Design.md` 的实现层深化，约束首次可运行版本（P0）的工程结构和验收标准。

### 1.1 P0 必须实现

- CarSim 状态进入 Simulink 后，能形成 `MeasBus`、`ErrBus` 和固定尺寸 `HistoryBus`。
- Transformer 使用保存好的网络和标准化参数在线推理，不执行训练、不更新权重。
- Risk Supervisor 输出有界、平滑的风险，并能在 Transformer 故障时切换到规则风险或安全保持值。
- 横向 MPC 至少支持 `Fixed MPC` 和 `Transformer-MPC` 两种模式；模式通过 `control_mode_id` 选择。
- 纵向速度外环独立跟踪 `v_ref_adapt`，不进入横向 MPC 优化变量。
- CarSim 能收到经过限幅、速率限制和有效性检查的控制命令。
- 每个 case 生成时间对齐的主日志和元数据，能够复跑同一场景。

### 1.2 明确不属于 P0

- 数据集生成、Transformer 训练、离线指标计算和论文作图脚本。
- 在线附着系数估计或真实附着系数输入。
- 端到端直接输出转角或加速度的神经网络控制器。
- 变量尺寸信号、字符串进入控制 Bus、运行时修改网络权重。

### 1.3 默认假设

| 项目 | P0 默认值 | 说明 |
|---|---:|---|
| MATLAB | R2023b 或更新版本 | 以 `dlnetwork`、Bus Editor 和 `matlab.unittest` 可用为前提 |
| CarSim | 2023.1 或更新版本 | 具体版本在 `case_meta.json` 中记录 |
| 控制周期 `Ts_mpc` | `0.02 s` | 所有高频控制子系统使用同一基准周期 |
| Transformer 周期 `Ts_tr` | `0.10 s` | 必须是 `Ts_mpc` 的整数倍，P0 为 5 倍 |
| 历史长度 `L` | `16` | 与原 Python demo 的 `history_len` 保持一致 |
| 特征数 `n_feature` | `14` | 列顺序见第 4 节，视为冻结接口 |
| MPC 预测步长 `N_p` | `15` | 预测时域为 `0.30 s` |
| MPC 控制步长 `N_c` | `5` | 决策变量为前 5 步转角增量 |
| 横向状态 | `[e_y,e_psi,beta,r]` | `r` 为横摆角速度，模型按 `v_x` 在线更新 |
| Transformer 执行设备 | CPU | P0 先保证确定性；GPU 仅作为可选优化 |

版本不满足上述假设时，不得直接修改闭环逻辑；先在 `config/tmpsim_config.m` 增加显式配置并更新启动检查。

## 2. 工程目录与文件责任

建议在当前工程目录下创建以下结构。目录名可以调整，但文件职责和接口名称必须保持不变。

```text
TransformerMPCSimulink/
├── model/
│   ├── tmpsim_online.slx                 # 唯一顶层闭环模型
│   ├── tmpsim_online.sldd                # Bus、枚举、参数数据字典
│   └── subsystems/                       # 可选的模型引用子系统
├── config/
│   ├── tmpsim_config.m                   # 生成全部 P0 参数
│   ├── vehicle_params.m                  # 车辆几何、质量、惯量、轮胎名义参数
│   ├── scenario_catalog.m                # 场景 ID、参考轨迹和低附着区间
│   ├── transformer_manifest.mat          # 网络、版本、输入输出签名
│   └── feature_stats.mat                 # mean_train、std_train、feature_order
├── src/
│   ├── +tmpsim/                          # MATLAB System 和纯函数
│   │   ├── ReferenceManager.m
│   │   ├── ErrorFeatureBuilder.m
│   │   ├── HistoryBuffer.m
│   │   ├── TransformerRiskSystem.m
│   │   ├── RiskSupervisor.m
│   │   ├── LateralMpcSystem.m
│   │   ├── SpeedOuterLoop.m
│   │   ├── CarSimAdapter.m
│   │   └── validateOnlineConfig.m
│   └── +tmpsim/+math/                     # 角度、限幅、滤波和离散化工具
│       ├── wrapAngle.m
│       ├── clipFinite.m
│       ├── updateAsymmetricFilter.m
│       └── discretizeBicycleModel.m
├── tests/
│   ├── TestSignalContracts.m
│   ├── TestErrorFeatureBuilder.m
│   ├── TestHistoryBuffer.m
│   ├── TestRiskSupervisor.m
│   ├── TestLateralMpcSystem.m
│   ├── TestSpeedOuterLoop.m
│   ├── TestTransformerRiskSystem.m
│   └── TestClosedLoopSmoke.m
├── cases/
│   ├── nominal_lane_change.json
│   ├── low_mu_lane_change.json
│   ├── sensor_fault.json
│   └── transformer_fault.json
├── scripts/
│   ├── setup_tmpsim.m                    # 加路径、加载字典、执行启动检查
│   ├── run_case.m                        # 单场景运行入口
│   └── run_regression_suite.m            # P0 回归入口，不做作图
└── logs/                                 # 运行后生成，不纳入源代码提交
```

`model/tmpsim_online.slx` 是唯一需要连接 CarSim 的模型。其余模块优先实现为 MATLAB System block，以便保存离散状态、固定端口尺寸并在模型测试中独立调用。

### 2.1 实现路线决策

| 路线 | 优点 | 代价 | 结论 |
|---|---|---|---|
| 纯 Simulink 标准块 + MPC Controller | 图形化直观，快速搭建 | 动态 Q/R、约束和 Transformer 推理接口受限 | 仅适合早期信号联调 |
| MATLAB System block + Data Dictionary + 固定维度 QP | 状态、类型、节拍和故障路径可显式控制 | 需要编写少量 MATLAB 类和单元测试 | **P0 采用**，兼顾可测试性与在线参数更新 |
| Python 外部进程/ROS 联调 | 与 Python 原型复用率高 | 通信延迟和运行环境不可控，难以证明实时性 | 不进入 P0，可作为离线对照 |

P0 先用第二条路线完成闭环；只有在单元测试和 10 秒 smoke test 通过后，才考虑将 Transformer 或 QP 替换为代码生成版本。

## 3. 顶层 Simulink 架构

顶层模型从左到右固定为以下 10 个子系统，信号只通过表中 Bus 传递：

```text
ScenarioManager -> ReferenceBus ------------------------------┐
                                                             v
CarSim Plant -> CarSimAdapter -> MeasBus -> ErrorFeatureBuilder -> ErrBus
                                             |                    |
                                             v                    v
                                        HistoryBuffer       LateralMPC
                                             |                    ^
                                             v                    |
                                      TransformerRisk -> RiskSupervisor
                                                               |   |
                                                               |   +-> SpeedOuterLoop -> CmdBus
                                                               +-----> LateralMPC -> CmdBus
                                                                              |
                                                                              v
                                                                    CarSimAdapter -> CarSim Plant
```

### 3.1 子系统端口

| 子系统 | 输入 | 输出 | 速率 |
|---|---|---|---:|
| `ScenarioManager` | `sim_time`、`reset`、`scenario_id` | `RefBus` | `Ts_mpc` |
| `CarSimAdapter`（测量侧） | CarSim 原始输出 | `MeasBus` | `Ts_mpc` |
| `ErrorFeatureBuilder` | `RefBus`、`MeasBus` | `ErrBus`、`latest_feature` | `Ts_mpc` |
| `HistoryBuffer` | `latest_feature`、`reset` | `HistoryBus` | `Ts_mpc` |
| `TransformerRisk` | `HistoryBus`、`transformer_enable`、`reset` | `RiskRawBus` | `Ts_mpc`，内部每 `Ts_tr` 更新 |
| `RiskSupervisor` | `RiskRawBus`、`ErrBus`、`MeasBus`、`RefBus.v_ref_base/ref_valid`、`reset` | `RiskBus`、`MPCParamBus`、`v_ref_adapt` | `Ts_mpc` |
| `LateralControllerVariant` | `ErrBus`、`MeasBus`、`RefBus`、`RiskBus`、`MPCParamBus` | `delta_cmd`、`mpc_feasible`、`mpc_status_code`、`solve_time_ms` | `Ts_mpc` |
| `SpeedOuterLoop` | `MeasBus.vx/meas_valid`、`v_ref_adapt`、`reset` | `ax_cmd`、`speed_status` | `Ts_mpc` |
| `CarSimAdapter`（命令侧） | `delta_cmd`、`ax_cmd`、有效性标志 | CarSim 控制输入 | `Ts_mpc` |
| `Logger` | 所有日志信号 | `sim_log.mat`、`sim_log.csv`、`case_meta.json` | `Ts_mpc` |

`RiskSupervisor` 必须同时接收 `ErrBus` 和 `MeasBus`，因为规则风险回退需要误差、侧偏角、横摆角速度和横向加速度；这是对原顶层设计中“规则回退”与输入列表之间缺口的实现性补充。

### 3.2 状态码冻结

| 字段 | 取值 | 含义 |
|---|---|---|
| `transformer_status_code` | `0/1/2/3` | reset/idle、running、ok、fallback |
| `supervisor_mode_id` | `0/1/2` | normal、fallback、recovering；仅作为日志字段 |
| `mpc_status_code` | `0/1/2/3/4/5/6` | not_run、solved、warm_start_solved、infeasible、timeout、invalid_input、fixed_mpc_degraded |
| `speed_status_code` | `0/1/2` | reset、normal、invalid_measurement |
| `control_mode_id` | `0/1/2/3` | PID、Fixed MPC、Rule-risk MPC、Transformer-MPC |

## 4. 冻结的数据契约

### 4.1 Bus 和基本类型

在 `model/tmpsim_online.sldd` 中定义以下 Bus：`RefBus`、`MeasBus`、`ErrBus`、`HistoryBus`、`RiskRawBus`、`RiskBus`、`MPCParamBus`、`CmdBus`。连续量为 `double`，标志位为 `boolean`，状态码为 `uint8`，索引为 `uint32` 或 `uint16`；所有数组固定尺寸。

Bus 字段沿用原设计第 7 节。P0 额外冻结以下字段：

| Bus | 必须字段 | 约束 |
|---|---|---|
| `RiskRawBus` | `r_low_raw,r_ey_raw,r_stab_raw,k_v_raw,risk_raw_valid,risk_source_id,transformer_status_code,transformer_latency_ms` | 推理失败时风险值仍输出上次值，但 `risk_raw_valid=false` |
| `RiskBus` | `r_low,r_ey,r_stab,k_v,risk_valid,degraded_mode,risk_source_id` | 所有风险和 `k_v` 已完成限幅和平滑 |
| `MPCParamBus` | 原文第 7.9 节全部字段 + `param_valid` | 参数无效时输出 baseline 参数，而不是 NaN |
| `CmdBus` | `delta_cmd,ax_cmd,cmd_valid,hold_last_cmd,control_mode_id` | `delta_cmd` 是前轮转角，`ax_cmd` 是加速度命令 |

### 4.2 特征列顺序

`feature_order` 必须作为 `feature_stats.mat` 中的只读元数据保存，并在启动时逐项比对：

```text
1 vx, 2 vy, 3 yaw_rate, 4 ay, 5 beta, 6 delta_meas,
7 delta_rate_meas, 8 ax_meas, 9 e_y, 10 e_psi,
11 e_y_rate, 12 e_psi_rate, 13 kappa_ref, 14 v_ref_base
```

启动检查必须拒绝以下情况：长度不为 14、名称不匹配、`std_train <= 0`、均值或标准差包含 `NaN/Inf`。

### 4.3 单位和符号约定

- 全局坐标使用 CarSim 输出的米制 `x/y`；误差在参考点 Frenet 坐标系中计算。
- `e_y > 0` 表示车辆位于参考轨迹左侧；`e_psi = wrapAngle(yaw - psi_ref)`。
- `delta > 0` 表示前轮左转；CarSim 的方向盘角只在 `CarSimAdapter` 中转换。
- 所有角度在内部使用弧度；CarSim 若输出角度制，适配层先转换。

## 5. 参数与模型附件

### 5.1 配置对象

`config/tmpsim_config.m` 返回一个结构体 `cfg`，并将关键项包装为 `Simulink.Parameter`：

```matlab
cfg.sample.Ts_mpc = 0.02;
cfg.sample.Ts_tr = 0.10;
cfg.history.L = 16;
cfg.history.n_feature = 14;
cfg.mpc.Np = 15;
cfg.mpc.Nc = 5;
cfg.mpc.delta_max = deg2rad(35);
cfg.mpc.delta_rate_max = deg2rad(450);   % rad/s
cfg.mpc.beta_max0 = deg2rad(8);
cfg.mpc.yaw_rate_max0 = 1.2;             % rad/s
cfg.mpc.ay_max0 = 6.0;                   % m/s^2
cfg.mpc.q_y0 = 12.0;
cfg.mpc.q_psi0 = 8.0;
cfg.mpc.q_beta0 = 4.0;
cfg.mpc.q_r0 = 2.0;
cfg.mpc.r_delta0 = 0.8;
cfg.mpc.r_d_delta0 = 0.2;
cfg.mpc.q_y_max = 60.0;
cfg.mpc.q_psi_max = 40.0;
cfg.mpc.q_beta_max = 30.0;
cfg.mpc.q_r_max = 20.0;
cfg.mpc.r_delta_max = 8.0;
cfg.mpc.r_d_delta_max = 5.0;
cfg.mpc.c_beta = 0.45;
cfg.mpc.c_yaw_rate = 0.35;
cfg.mpc.c_ay = 0.30;
cfg.mpc.beta_min = deg2rad(2.0);
cfg.mpc.yaw_rate_min = 0.4;
cfg.mpc.ay_min = 2.5;
cfg.mpc.delta_rate_min = deg2rad(120);
cfg.risk.k_v_min = 0.35;
cfg.risk.rho_up = 0.25;
cfg.risk.rho_down = 0.90;
cfg.risk.fail_cycles = 3;
cfg.risk.recover_cycles = 10;
cfg.speed.v_min = 1.0;                   % m/s
cfg.speed.a_min = -6.0;                  % m/s^2
cfg.speed.a_max = 2.5;                   % m/s^2
cfg.speed.jerk_max = 8.0;                % m/s^3
```

权重基线、上下界、规则风险阈值和车辆参数也必须集中在 `cfg` 中，不得散落在 MATLAB Function block 的常量表达式中。上述数值是 P0 初值，正式实验前只能通过配置文件调参。

### 5.2 Transformer 附件

`config/transformer_manifest.mat` 至少包含：

```text
net              dlnetwork 或兼容 predict 接口的网络对象
model_version    char 或 string，仅用于元数据，不进入 Bus
input_size       [16 14]
output_names     {'r_low','r_ey','r_stab','k_v'}
output_bounds    [0 1; 0 1; 0 1; k_v_min 1]
```

`config/feature_stats.mat` 至少包含：`mean_train`（`1x14 double`）、`std_train`（`1x14 double`）、`feature_order`（14 个固定名称）。模型附件只在 `setupImpl` 或模型初始化阶段加载一次。

### 5.3 场景 JSON 最小格式

每个 `cases/*.json` 至少包含以下字段；故障字段缺省时视为关闭：

```json
{
  "scenario_id": "low_mu_lane_change",
  "duration_s": 10.0,
  "control_mode_id": 3,
  "random_seed": 20260904,
  "faults": {
    "transformer_nan_from_step": -1,
    "sensor_inf_step": -1,
    "force_mpc_infeasible": false
  }
}
```

`run_case` 只读取 JSON 并设置配置，不在场景文件中写入 Bus 字段或网络参数；这样可以防止实验场景绕过接口校验。

## 6. 各模块实现细则

### 6.1 ReferenceManager

实现文件：`src/+tmpsim/ReferenceManager.m`。

- 从 `scenario_catalog` 读取固定尺寸参考路径数组 `path_xypsi_kappa_v`。
- 用单调不回退的 `path_idx` 搜索最近点；每个周期最多向前搜索 `search_window=50` 个点。
- 车辆离开路径末端时锁定末点并置 `ref_valid=false`，不得数组越界。
- 输出 `v_ref_base` 前先执行场景速度上限和 `cfg.speed.v_min` 限制。
- `reset=true` 时将索引置为 1。

验收：参考点索引不回退、末端不越界、角度连续、`ref_valid=false` 时下游进入安全策略。

### 6.2 ErrorFeatureBuilder

实现文件：`src/+tmpsim/ErrorFeatureBuilder.m`。

计算规则：

```text
dx = x - x_ref
dy = y - y_ref
e_y = -sin(psi_ref) * dx + cos(psi_ref) * dy
e_psi = wrapAngle(yaw - psi_ref)
e_y_rate = vy + vx * sin(e_psi)
e_psi_rate = yaw_rate - vx * kappa_ref
```

其中 `e_y_rate` 使用测量横向速度和航向误差的几何关系，避免直接对带噪声的 `e_y` 做差分；若 `vx < 0.5 m/s`，改用内部状态 `e_y_prev` 计算 `clip((e_y-e_y_prev)/Ts_mpc, -5, 5)`，并将 `err_valid=false`。`e_y_prev` 在每次有效更新后写回，复位时清零。

`latest_feature` 必须按第 4.2 节顺序逐列写入，输出前检查所有元素有限。

### 6.3 HistoryBuffer

实现文件：`src/+tmpsim/HistoryBuffer.m`。

- 内部状态为 `buffer[16x14]`、`write_idx`、`history_len`。
- 每个 `Ts_mpc` 写入一行；写满后按环形队列覆盖最旧样本。
- 输出 `window` 时按时间从旧到新排列，不能直接暴露环形物理存储顺序。
- `history_len < 16` 时允许输出零填充窗口，但 `window_ready=false`。
- `reset=true` 时清零 buffer 和计数器。

### 6.4 TransformerRiskSystem

实现文件：`src/+tmpsim/TransformerRiskSystem.m`，使用 MATLAB System block。

每个高频周期执行以下步骤：

1. 若 `reset`，清空上次风险并置 `status_code=0`。
2. 若 `transformer_enable=false` 或尚未到 `Ts_tr` 节拍，保持上次风险，状态码置 `1`（idle）。
3. 检查 `window_ready`、有限性和尺寸；失败则输出上次值并置 `risk_raw_valid=false`、状态码 `3`（fallback）。
4. 计算 `x_norm=(window-mean_train)./std_train`，调用一次 `predict(net,x_norm)`。
5. 将输出按固定顺序解包并限幅到 `output_bounds`，记录 `transformer_latency_ms`。
6. 只有推理成功且输出有限时才置 `risk_raw_valid=true`、状态码 `2`（ok）。

P0 的节拍器使用 `mod(step_count, Ts_tr/Ts_mpc)==0`，并在启动检查中确认比值为整数。推理不允许在 `stepImpl` 内加载文件或分配不受控的变量尺寸数组。

### 6.5 RiskSupervisor

实现文件：`src/+tmpsim/RiskSupervisor.m`。

#### 规则风险回退

当 Transformer 无效时，由当前 `ErrBus` 和 `MeasBus` 计算：

```text
r_ey_rule   = clip(abs(e_y) / e_y_warn, 0, 1)
r_stab_rule = max(clip(abs(beta) / beta_warn, 0, 1),
                   clip(abs(yaw_rate) / yaw_rate_warn, 0, 1),
                   clip(abs(ay) / ay_warn, 0, 1))
r_low_rule  = clip(0.5*r_ey_rule + 0.5*r_stab_rule, 0, 1)
k_v_rule    = clip(1 - c_v_rule*r_low_rule, k_v_min, 1)
```

P0 阈值取 `e_y_warn=0.30 m`、`beta_warn=5 deg`、`yaw_rate_warn=0.8 rad/s`、`ay_warn=4.0 m/s^2`、`c_v_rule=0.50`，全部来自 `cfg.risk`。

#### 有效性状态机

| 模式 | 进入条件 | 输出来源 | 退出条件 |
|---|---|---|---|
| `normal=0` | Transformer 连续有效 | Transformer | 连续 `fail_cycles=3` 次无效 |
| `fallback=1` | 连续 3 次无效 | 规则风险 | 连续 `recover_cycles=10` 次有效 |
| `recovering=2` | 从 fallback 收到有效输出 | Transformer 风险按平滑系数恢复 | 连续 10 次有效后回 normal；期间再次失败回 fallback |

若规则风险也无效，使用 `last_good_risk` 和 `last_good_params`；若连续无有效输入超过 `2*fail_cycles`，固定输出 baseline 参数并置 `degraded_mode=true`。

#### 限幅、平滑和映射

先限幅，再采用非对称一阶滤波：

```text
rho = rho_up   if r_raw > r_prev else rho_down
r_filt = rho*r_prev + (1-rho)*r_raw
```

权重、约束和速度映射沿用原设计第 10.6 节公式。映射后必须执行 `param_valid` 检查：所有权重为有限正数、下界不超过上界、`v_ref_adapt` 位于 `[v_min, v_ref_base]`。检查失败时输出 baseline 参数并进入 `degraded_mode`。

为使实现文件不依赖原文上下文，P0 直接采用以下映射：

```text
q_y       = clip(q_y0       * (1 + 1.2*r_ey),   q_y0,       q_y_max)
q_psi     = clip(q_psi0     * (1 + 1.0*r_ey),   q_psi0,     q_psi_max)
q_beta    = clip(q_beta0    * (1 + 2.4*r_stab), q_beta0,    q_beta_max)
q_r       = clip(q_r0       * (1 + 2.0*r_stab), q_r0,       q_r_max)
r_delta   = clip(r_delta0   * (1 + 1.2*r_stab), r_delta0,   r_delta_max)
r_d_delta = clip(r_d_delta0 * (1 + 2.0*r_stab), r_d_delta0, r_d_delta_max)
beta_max  = clip(beta_max0  * (1 - c_beta*r_stab), beta_min, beta_max0)
yaw_rate_max = clip(yaw_rate_max0 * (1 - c_yaw_rate*r_stab), yaw_rate_min, yaw_rate_max0)
ay_max    = clip(ay_max0    * (1 - c_ay*r_stab), ay_min, ay_max0)
delta_rate_max = clip(delta_rate_max0 * (1 - 0.45*r_stab), delta_rate_min, delta_rate_max0)
speed_scale = min(k_v, 1 - 0.50*r_low)
v_ref_adapt = v_ref_base * clip(speed_scale, k_v_min, 1)
```

当 `RefBus.ref_valid=false` 时，Risk Supervisor 强制将 `v_ref_adapt` 置为 `cfg.speed.v_min`，并将 `degraded_mode=true`；风险值仍按当前有效来源记录。

### 6.6 LateralMpcSystem

实现文件：`src/+tmpsim/LateralMpcSystem.m`。

#### 模型和预测

使用名义线性单轨模型，状态为 `x=[e_y,e_psi,beta,r]`，输入为前轮转角增量 `d_delta`，曲率作为已知前馈项。每个 `Ts_mpc` 根据当前 `vx` 更新连续模型并离散化：

```text
vx_used = max(vx, 1.0)
[A_d,B_d,E_d] = discretizeBicycleModel(vehicle,cfg,vx_used,Ts_mpc)
```

预测步数 `N_p=15`，控制步数 `N_c=5`。代价矩阵由 `MPCParamBus` 动态生成：

```text
Q = diag([q_y, q_psi, q_beta, q_r])
R = diag([r_delta, r_d_delta])
```

实现时将上一时刻转角作为增广状态，以便同时约束 `delta` 和 `d_delta`。参考曲率的前馈转角使用：

```text
delta_ff = atan((lf + lr) * kappa_ref)
```

最终命令必须满足：

```text
delta_cmd = clip(delta_ff + d_delta_0,
                  delta_prev - delta_rate_max*Ts_mpc,
                  delta_prev + delta_rate_max*Ts_mpc)
delta_cmd = clip(delta_cmd, -delta_max, delta_max)
```

横向加速度约束不直接使用未知附着系数，而是从当前离散模型得到线性预测关系 `a_y = C_ay*x + D_ay*d_delta + d_ay`，对每个预测步施加 `-ay_max <= a_y <= ay_max`。`C_ay` 和 `D_ay` 在 `discretizeBicycleModel` 中由名义车辆参数和 `vx_used` 计算。

#### 求解器和状态码

P0 使用固定维度稠密 QP 和 `mpcActiveSetSolver`；若本地没有 MPC Toolbox，允许在 MATLAB-only 单元测试中用 `quadprog` 验证矩阵，但不得将其作为闭环交付求解器。

`mpc_status_code` 冻结为：`0=not_run`、`1=solved`、`2=warm_start_solved`、`3=infeasible`、`4=timeout`、`5=invalid_input`、`6=fixed_mpc_degraded`。

求解失败处理：当前周期保持上一时刻可行命令并置 `hold_last_cmd=true`；连续 3 次失败后用 baseline 参数再次求解；baseline 仍失败则输出 `delta_ff` 的限幅值，`cmd_valid=false`，由 `CarSimAdapter` 进入保持策略。

### 6.7 SpeedOuterLoop

实现文件：`src/+tmpsim/SpeedOuterLoop.m`。

采用带抗饱和和加加速度限制的 PI：

```text
e_v = v_ref_adapt - vx
ax_unsat = kp_v*e_v + ki_v*integral_e_v
ax_cmd = clip(ax_unsat, a_min, a_max)
ax_cmd = clip(ax_cmd, ax_prev-jerk_max*Ts_mpc, ax_prev+jerk_max*Ts_mpc)
```

当 `ax_unsat` 超出加速度边界时，积分器只按反向误差更新；`reset=true` 或 `MeasBus.meas_valid=false` 时积分器清零。输出只包含 `ax_cmd`，不把 throttle/brake 细节泄漏到横向控制器。

### 6.8 CarSimAdapter

实现文件：`src/+tmpsim/CarSimAdapter.m`。

- 建立一张显式的 CarSim 信号映射表，包含原始名称、单位、方向和转换系数。
- 测量侧完成角度制到弧度制、方向盘角到前轮角的转换，并对有限性进行检查。
- 命令侧将 `ax_cmd >= 0` 映射为 throttle、`ax_cmd < 0` 映射为 brake；两者互斥且均限幅到 `[0,1]`。
- `cmd_valid=false` 或 `hold_last_cmd=true` 时输出上一次经过限幅的有效命令；仿真复位时输出零转角和零加速度。
- 记录适配前后信号，便于定位 CarSim 端的符号或单位错误。

## 7. 时序、复位和数据一致性

### 7.1 速率调度

- 顶层基准采样时间为 `Ts_mpc=0.02 s`。
- Transformer 子系统仍以基准速率触发，但仅在每 5 个基准周期执行推理；两次推理之间保持上次有效风险。
- CarSim、误差构造、历史缓存、Risk Supervisor、MPC 和速度外环均按 `Ts_mpc` 执行。
- 不使用隐式 Rate Transition；跨速率信号通过显式 `Unit Delay/Memory` 和固定采样保持。

### 7.2 单周期执行顺序

同一周期内严格按以下顺序更新：

1. 读取 CarSim 测量和场景参考量。
2. 计算误差并追加当前特征到历史缓存。
3. 若到 Transformer 节拍，先读取完整历史窗口并更新 `RiskRawBus`。
4. Risk Supervisor 使用本周期最新风险或上次保持风险。
5. 横向 MPC 使用当前 `ErrBus` 和 `MPCParamBus` 求解。
6. 速度外环使用当前 `v_ref_adapt` 求解。
7. CarSimAdapter 统一限幅并输出控制命令。
8. Logger 在命令输出后记录本周期所有信号。

### 7.3 复位语义

`reset` 来自模型初始化或仿真重新开始，仅在一个基准周期内有效。所有状态块都必须响应同一 `reset`：`path_idx`、历史缓存、Transformer 节拍器、风险滤波器、回退计数器、MPC 上次命令和速度积分器。

## 8. 日志与可追溯性

### 8.1 主日志字段

主日志沿用原设计第 8.2 节全部字段，并追加：

| 字段 | 类型 | 用途 |
|---|---|---|
| `step_idx` | `uint32` | 与 `time` 双重定位样本 |
| `transformer_status_code` | `uint8` | 区分 idle/running/ok/fallback |
| `transformer_latency_ms` | `double` | 推理耗时 |
| `mpc_status_code` | `uint8` | 求解器状态 |
| `speed_status_code` | `uint8` | 速度外环状态 |
| `fallback_counter` | `uint16` | 风险连续失效计数 |
| `recovery_counter` | `uint16` | 风险恢复计数 |
| `delta_ff` | `double` | MPC 曲率前馈转角 |
| `throttle_cmd` | `double` | CarSim 适配后的油门 |
| `brake_cmd` | `double` | CarSim 适配后的制动 |

所有记录信号都用同一 `step_idx` 写入；禁止各子系统各自创建独立时间轴。

### 8.2 元数据

每个 case 生成 `case_meta.json`，至少记录：场景 ID、控制模式、MATLAB/Simulink/CarSim 版本、Git commit（若有）、模型版本、`Ts_mpc`、`Ts_tr`、`L`、`Np/Nc`、车辆参数哈希、Transformer manifest 哈希、运行开始时间和随机种子。

当前目录不是 Git 仓库时，`git_commit` 写为 `not_a_git_repository`，不得伪造版本号。

## 9. 测试与验收矩阵

### 9.1 单元测试

使用 `matlab.unittest`，每个纯函数和 MATLAB System block 至少覆盖以下边界：

| 测试文件 | 必须验证 |
|---|---|
| `TestSignalContracts.m` | Bus 字段名、类型、维度、单位元数据和特征顺序完全匹配 |
| `TestErrorFeatureBuilder.m` | 直线零误差、左侧正误差、航向跨 `+-pi`、低速有效性 |
| `TestHistoryBuffer.m` | 未填满窗口、填满窗口、环形覆盖、reset 清空 |
| `TestRiskSupervisor.m` | 限幅、上升快/下降慢滤波、规则回退、3 次失败、10 次恢复、参数可行性 |
| `TestLateralMpcSystem.m` | nominal 求解、动态权重、转角/速率约束、不可行回退、状态码 |
| `TestSpeedOuterLoop.m` | 加速度饱和、抗积分饱和、jerk 限制、reset |
| `TestTransformerRiskSystem.m` | 标准化顺序、窗口未就绪、异常输出限幅、保持上次值、推理节拍 |

### 9.2 模型级测试

`TestClosedLoopSmoke.m` 运行 `cases/nominal_lane_change.json`，仿真至少 `10 s`，验收：

- 仿真正常结束，无 MATLAB 警告升级为错误。
- 任何记录字段都不存在 `NaN` 或 `Inf`。
- `time` 严格递增，间隔等于 `Ts_mpc`。
- `mpc_status_code` 不出现 `5`，且 `mpc_feasible` 在正常工况下为真。
- `solve_time_ms` 的 95 分位不超过 `0.8*Ts_mpc*1000 = 16 ms`。
- Transformer 有效运行时 `risk_source_id=1`，未就绪期间不允许风险作为有效控制依据。

### 9.3 故障注入测试

| 场景 | 注入 | 预期 |
|---|---|---|
| `transformer_fault` | 连续 5 次输出 `NaN` | 第 3 次后进入 fallback，使用规则风险，不出现 NaN |
| `sensor_fault` | `vy` 或 `yaw_rate` 置为 `Inf` 1 个周期 | `meas_valid=false`，命令保持上一有效值，随后恢复 |
| `mpc_infeasible` | 将 `delta_max` 临时降至 0 | 状态码为 3/6，控制链不中断，日志记录降级 |
| `reference_end` | 参考索引到达末点 | 不越界，`ref_valid=false`，速度降至 `v_min` |

### 9.4 对比验收

同一场景、同一初始条件下运行 `PID`、`Fixed MPC`、`Rule-risk MPC`、`Transformer-MPC`。P0 只要求闭环可复现并具备非退化证据：

- `Transformer-MPC` 的 RMS 横向误差不超过 `Fixed MPC` 的 `1.10` 倍。
- `Transformer-MPC` 的最大 `|beta|` 不超过 `Fixed MPC` 的 `1.10` 倍。
- 低附着场景中，至少一个安全指标（最大 `|beta|`、最大 `|ay|` 或超限持续时间）优于 `Fixed MPC`，并能在日志中定位风险触发与参数变化的先后关系。

## 10. 分阶段实施任务

### Task 1：建立参数和数据字典

**Files:**
- Create: `config/tmpsim_config.m`
- Create: `config/vehicle_params.m`
- Create: `config/scenario_catalog.m`
- Create: `model/tmpsim_online.sldd`
- Create: `src/+tmpsim/validateOnlineConfig.m`
- Test: `tests/TestSignalContracts.m`

- [ ] 定义第 1 节全部默认参数和车辆参数。
- [ ] 在 Data Dictionary 中创建 8 个 Bus，并为每个元素设置名称、固定维度、数据类型和采样时间。
- [ ] 为 `feature_order`、状态码和 `control_mode_id` 创建枚举或常量表。
- [ ] 实现 `validateOnlineConfig(cfg)`，检查 `Ts_tr/Ts_mpc` 为整数、`L=16`、特征统计量合法、约束上下界一致。
- [ ] 运行 `results = runtests('tests/TestSignalContracts.m'); assertSuccess(results);`，预期全部通过。

### Task 2：实现参考、测量和特征链

**Files:**
- Create: `src/+tmpsim/ReferenceManager.m`
- Create: `src/+tmpsim/ErrorFeatureBuilder.m`
- Create: `src/+tmpsim/CarSimAdapter.m`
- Test: `tests/TestErrorFeatureBuilder.m`

- [ ] 实现参考点单调搜索、末点处理和 `RefBus` 输出。
- [ ] 实现 Frenet 误差、角度包络、低速有效性和固定 14 列特征写入。
- [ ] 建立 CarSim 原始信号到 `MeasBus` 的显式单位/符号映射。
- [ ] 运行 `results = runtests('tests/TestErrorFeatureBuilder.m'); assertSuccess(results);`，预期零失败。

### Task 3：实现历史窗口

**Files:**
- Create: `src/+tmpsim/HistoryBuffer.m`
- Test: `tests/TestHistoryBuffer.m`

- [ ] 实现 `16x14` 固定尺寸环形缓存和时间正序输出。
- [ ] 实现 startup zero-fill、`window_ready` 和统一 reset。
- [ ] 运行 `results = runtests('tests/TestHistoryBuffer.m'); assertSuccess(results);`，预期覆盖未就绪、填满、覆盖、复位四类用例。

### Task 4：接入 Transformer 在线推理

**Files:**
- Create: `config/transformer_manifest.mat`
- Create: `config/feature_stats.mat`
- Create: `src/+tmpsim/TransformerRiskSystem.m`
- Test: `tests/TestTransformerRiskSystem.m`

- [ ] 导出并冻结网络输入尺寸 `[16,14]`、输出顺序和边界。
- [ ] 在 `setupImpl` 加载网络和标准化参数，在 `stepImpl` 实现节拍、标准化、推理、限幅和状态码。
- [ ] 测量 1000 次推理延迟，保存均值、95 分位和最大值到测试输出。
- [ ] 运行 `results = runtests('tests/TestTransformerRiskSystem.m'); assertSuccess(results);`，预期异常输入、窗口未就绪、节拍保持用例全部通过。

### Task 5：实现 Risk Supervisor

**Files:**
- Create: `src/+tmpsim/RiskSupervisor.m`
- Create: `src/+tmpsim/+math/clipFinite.m`
- Create: `src/+tmpsim/+math/updateAsymmetricFilter.m`
- Test: `tests/TestRiskSupervisor.m`

- [ ] 实现 Transformer 风险与规则风险的选择、限幅、非对称滤波和状态机。
- [ ] 实现第 6.5 节的参数映射、可行性检查和 baseline 回退。
- [ ] 运行 `results = runtests('tests/TestRiskSupervisor.m'); assertSuccess(results);`，预期状态转换和参数边界全部通过。

### Task 6：实现横向 MPC

**Files:**
- Create: `src/+tmpsim/+math/discretizeBicycleModel.m`
- Create: `src/+tmpsim/LateralMpcSystem.m`
- Test: `tests/TestLateralMpcSystem.m`

- [ ] 实现名义单轨模型、在线离散化、固定维度 QP 矩阵和 warm start。
- [ ] 接入 `mpcActiveSetSolver`，实现动态权重、约束、前馈转角和失败回退。
- [ ] 运行 `results = runtests('tests/TestLateralMpcSystem.m'); assertSuccess(results);`，预期 nominal、动态参数、不可行和限幅用例通过。

### Task 7：实现纵向外环和命令适配

**Files:**
- Create: `src/+tmpsim/SpeedOuterLoop.m`
- Modify: `src/+tmpsim/CarSimAdapter.m`
- Test: `tests/TestSpeedOuterLoop.m`

- [ ] 实现带抗积分饱和和 jerk 限制的 PI 速度外环。
- [ ] 将 `ax_cmd` 转换为互斥 throttle/brake，并实现无效命令保持策略。
- [ ] 运行 `results = runtests('tests/TestSpeedOuterLoop.m'); assertSuccess(results);`，预期饱和、复位和故障保持用例通过。

### Task 8：组装顶层模型和日志

**Files:**
- Create: `model/tmpsim_online.slx`
- Create: `scripts/setup_tmpsim.m`
- Create: `scripts/run_case.m`
- Create: `scripts/run_regression_suite.m`
- Test: `tests/TestClosedLoopSmoke.m`

- [ ] 按第 3 节顺序连接 10 个子系统，显式设置采样时间和 Bus 类型。
- [ ] 接入 CarSim S-Function/接口，确认测量侧和命令侧信号都经过 `CarSimAdapter`。
- [ ] 使用 `Dataset` 或 `To Workspace` 只建立一条统一日志时间轴，并输出 CSV、MAT 和 JSON。
- [ ] 运行 `results = runtests('tests/TestClosedLoopSmoke.m'); assertSuccess(results);`，预期 10 秒 nominal case 无 NaN/Inf、无未处理错误。

### Task 9：故障注入和四模式回归

**Files:**
- Create: `cases/transformer_fault.json`
- Create: `cases/sensor_fault.json`
- Create: `cases/low_mu_lane_change.json`
- Modify: `scripts/run_regression_suite.m`

- [ ] 执行 nominal、low-mu、Transformer fault、sensor fault、MPC infeasible、reference end 六类 case。
- [ ] 对每个 case 保存 `case_meta.json` 和主日志，确保场景、模式、版本和参数哈希齐全。
- [ ] 对四种控制模式执行同一初始条件对比，输出原始数值摘要，不生成论文图。
- [ ] 只有第 9 节全部验收条件满足时，才标记 P0 完成。

## 11. 启动、运行和回归命令

在 MATLAB 当前目录切换到 `TransformerMPCSimulink/` 后执行：

```matlab
run('scripts/setup_tmpsim.m');
validateOnlineConfig(cfg);

result = run_case('cases/nominal_lane_change.json', ...
                  'control_mode_id', uint8(3));
assert(result.completed == true);

summary = run_regression_suite();
assert(summary.all_passed == true);
```

`setup_tmpsim.m` 必须完成：加入 `src/` 路径、加载 `tmpsim_online.sldd`、加载配置和模型附件、检查工具箱版本、调用 `validateOnlineConfig`。任何检查失败都在仿真开始前报错，不允许运行到半程才发现配置问题。

## 12. 风险清单与处理顺序

| 风险 | 早期信号 | 处理顺序 |
|---|---|---|
| CarSim 单位/符号错 | 直线工况误差发散、左转方向反向 | 先只运行 Adapter + ErrorFeatureBuilder，检查 10 个固定工况 |
| Transformer 输入漂移 | `risk_raw_valid=false`、输出长期贴边 | 比对 `feature_order`、均值标准差和 Python 单步输出 |
| MPC 求解超时 | `solve_time_ms` 接近周期上限 | 先减小 `Np/Nc` 或改用 warm start，不先放宽安全约束 |
| 参数映射导致不可行 | `param_valid=false` 或连续 infeasible | 检查约束上下界，再检查风险映射系数，最后才调 baseline |
| 速率不同步 | 风险曲线和控制参数错位 1 到数个采样点 | 检查节拍计数、Unit Delay 和 Logger 的统一 `step_idx` |
| 故障保持不安全 | 传感器故障时命令突跳 | Adapter 统一限幅、保持上一有效命令并记录 `hold_last_cmd` |

排查顺序固定为：接口/单位 -> 时序/复位 -> 模型数值 -> 求解器性能 -> 参数调优。不要在接口尚未验证前调 Transformer 或 MPC 权重。

## 13. 交付检查表

- [ ] `tmpsim_online.slx` 能在干净 MATLAB 会话中由 `setup_tmpsim.m` 加载。
- [ ] 8 个 Bus 的字段、类型、维度与本文件一致，特征顺序检查可自动失败。
- [ ] Transformer、Risk Supervisor、MPC、速度外环均能脱离 CarSim 完成单元测试。
- [ ] nominal 和 low-mu case 的主日志、元数据和版本信息齐全。
- [ ] Transformer 故障、传感器故障、MPC 不可行和参考轨迹结束均有可观测降级状态。
- [ ] nominal case 的 Transformer 推理和 MPC 求解 p95 延迟分别低于 `Ts_tr` 和 `0.8*Ts_mpc`。
- [ ] 四种控制模式使用相同场景配置和初始条件，能够复现并进行数值比较。
- [ ] 原始顶层设计文档仍保留；本文件作为实现层唯一执行入口。
