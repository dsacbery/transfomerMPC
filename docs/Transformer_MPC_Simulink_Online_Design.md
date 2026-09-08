# Transformer-MPC Simulink 在线闭环顶层设计

## 1. 目标

本文件记录面向 MATLAB/Simulink + CarSim 的在线闭环工程顶层设计。

工程只负责：

- 在线状态采集
- 在线风险推理
- 在线风险到控制参数映射
- 横向 MPC 控制
- 纵向速度外环控制
- CarSim 闭环仿真与日志记录

工程不包含：

- 数据集生成
- Transformer 训练
- 离线评估脚本
- 作图脚本
- 论文分析脚本

## 2. 已确认的控制边界

- 仿真平台：CarSim-Simulink 联合仿真
- 控制形式：联合控制，但按“横向 MPC + 纵向速度外环”实现
- 横向控制：MPC 为主
- 纵向控制：独立速度外环，不并入 MPC 优化
- 风险输入：仅使用车辆自身状态与轨迹误差时序，不使用真实附着系数在线输入
- Transformer 输出：风险，不直接输出控制量

## 3. 顶层模块划分

### 3.1 Reference & Scenario Manager

职责：

- 生成参考轨迹
- 输出参考曲率、参考航向、参考速度
- 配置场景参数

输出：

- `x_ref`
- `y_ref`
- `psi_ref`
- `kappa_ref`
- `v_ref_base`

### 3.2 Error & Feature Builder

职责：

- 根据 CarSim 车辆状态与参考轨迹计算误差
- 组装单步特征向量

输出：

- `e_y`
- `e_psi`
- `e_y_rate`
- `e_psi_rate`
- `feature_vec`

### 3.3 History Buffer

职责：

- 缓存最近 `L` 步特征
- 形成 Transformer 输入窗口

输出：

- `window[L x n_feature]`

### 3.4 Transformer Risk Prediction

职责：

- 读取时序窗口
- 输出短时风险与速度修正因子

输出：

- `r_low`
- `r_ey`
- `r_stab`
- `k_v`

### 3.5 Risk Supervisor

职责：

- 风险限幅
- 风险平滑
- 模型失效时降级
- 风险到 MPC 参数的中间映射

输出：

- `MPCParamBus`
- `v_ref_adapt`

### 3.6 Lateral Controller Variant

职责：

- 承载横向控制器变体
- 便于切换 PID / Fixed MPC / Rule-risk MPC / Transformer-MPC

输出：

- `delta_cmd`
- `mpc_feasible`
- `mpc_status`
- `solve_time`

### 3.7 Speed Outer Loop

职责：

- 纵向速度控制
- 跟踪 `v_ref_adapt`

输出：

- `ax_cmd` 或 `throttle_cmd / brake_cmd`

### 3.8 CarSim Interface & Logger

职责：

- 将控制量送入 CarSim
- 记录状态、误差、风险、控制和求解信息

输出：

- 车辆状态
- 日志信号
- 评估原始数据

## 4. 推荐的 Bus 命名

- `RefBus`
- `MeasBus`
- `ErrBus`
- `HistoryBus`
- `RiskRawBus`
- `RiskBus`
- `MPCParamBus`
- `CmdBus`

## 5. 参考 Python 代码映射

在线工程对应 Python 仓库中的核心部分：

- `reference_path`
- `tracking_error`
- `vehicle_model`
- `transformer_model`
- `risk_mapper`
- `mpc_solver`
- `simulator`

不进入在线工程主链：

- `dataset`
- `train_transformer`
- `advantage_analysis`
- `plotting`
- `metrics`
- `tests`

## 6. 下一步要细化的内容

优先细化第 1 项：

- `Bus/信号字典`

需要把每个 Bus 的字段、单位、维度、数据类型、采样来源和使用模块全部定义清楚。

## 7. Bus/信号字典

### 7.1 统一约束

- 所有 Bus 元素名统一使用 `snake_case`
- 所有连续量默认使用 `double`
- 所有标志位使用 `boolean`
- 所有模式/状态码使用 `uint8`
- 所有数组使用固定尺寸，避免变量尺寸信号进入主闭环
- Bus 内不使用字符串
- 内部控制量统一使用前轮转角 `delta`，CarSim 接口处再做方向盘角转换
- 表中对连续量写物理单位，对离散量直接写数据类型

### 7.2 RefBus

来源：`Reference & Scenario Manager`

消费者：`Error & Feature Builder`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `x_ref` | 1x1 | m | 参考路径当前点纵向坐标 |
| `y_ref` | 1x1 | m | 参考路径当前点横向坐标 |
| `psi_ref` | 1x1 | rad | 参考航向角 |
| `kappa_ref` | 1x1 | 1/m | 参考曲率 |
| `v_ref_base` | 1x1 | m/s | 基础参考速度 |
| `path_idx` | 1x1 | uint32 | 当前最近参考点索引 |
| `ref_valid` | 1x1 | boolean | 参考路径是否有效 |

### 7.3 MeasBus

来源：`CarSim Plant`

消费者：`Error & Feature Builder`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `x` | 1x1 | m | 车辆全局坐标 x |
| `y` | 1x1 | m | 车辆全局坐标 y |
| `yaw` | 1x1 | rad | 车辆航向角 |
| `vx` | 1x1 | m/s | 纵向速度 |
| `vy` | 1x1 | m/s | 横向速度 |
| `yaw_rate` | 1x1 | rad/s | 横摆角速度 |
| `beta` | 1x1 | rad | 质心侧偏角 |
| `ay` | 1x1 | m/s^2 | 横向加速度 |
| `delta_meas` | 1x1 | rad | 实际前轮转角 |
| `ax_meas` | 1x1 | m/s^2 | 实际纵向加速度 |
| `delta_rate_meas` | 1x1 | rad/s | 实际转角变化率 |
| `meas_valid` | 1x1 | boolean | 测量是否有效 |

### 7.4 ErrBus

来源：`Error & Feature Builder`

消费者：`History Buffer`、`Lateral Controller Variant`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `e_y` | 1x1 | m | 横向误差 |
| `e_psi` | 1x1 | rad | 航向误差 |
| `e_y_rate` | 1x1 | m/s | 横向误差变化率 |
| `e_psi_rate` | 1x1 | rad/s | 航向误差变化率 |
| `yaw_ref` | 1x1 | rad | 参考航向角 |
| `kappa_ref` | 1x1 | 1/m | 当前参考曲率 |
| `v_ref_base` | 1x1 | m/s | 当前基础参考速度 |
| `err_valid` | 1x1 | boolean | 误差是否有效 |

### 7.5 HistoryBus

来源：`History Buffer`

消费者：`Transformer Risk Prediction`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `window` | Lx14 | `double` | 最近 L 步原始特征窗口 |
| `latest_feature` | 1x14 | `double` | 当前步原始特征向量 |
| `window_ready` | 1x1 | boolean | 窗口是否已经填满 |
| `history_len` | 1x1 | uint16 | 当前历史窗口长度 |
| `sample_time` | 1x1 | s | 采样周期 |

说明：

- `window` 的列顺序必须固定，不允许后续随意调换
- `L` 初版建议直接沿用 Python demo 的 `history_len = 16`

### 7.6 特征向量顺序

`latest_feature` 和 `window` 每一行的字段顺序固定为：

| 序号 | 字段 | 来源 |
|---|---|---|
| 1 | `vx` | `MeasBus` |
| 2 | `vy` | `MeasBus` |
| 3 | `yaw_rate` | `MeasBus` |
| 4 | `ay` | `MeasBus` |
| 5 | `beta` | `MeasBus` |
| 6 | `delta_meas` | `MeasBus` |
| 7 | `delta_rate_meas` | `MeasBus` |
| 8 | `ax_meas` | `MeasBus` |
| 9 | `e_y` | `ErrBus` |
| 10 | `e_psi` | `ErrBus` |
| 11 | `e_y_rate` | `ErrBus` |
| 12 | `e_psi_rate` | `ErrBus` |
| 13 | `kappa_ref` | `ErrBus` |
| 14 | `v_ref_base` | `ErrBus` |

说明：

- 这里的 `ax_meas`、`delta_meas`、`delta_rate_meas` 指车辆实际响应，不是命令值
- 这一顺序后续不得在训练和在线推理之间发生变化

### 7.7 RiskRawBus

来源：`Transformer Risk Prediction`

消费者：`Risk Supervisor`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `r_low_raw` | 1x1 | - | 原始综合风险 |
| `r_ey_raw` | 1x1 | - | 原始横向误差风险 |
| `r_stab_raw` | 1x1 | - | 原始稳定性风险 |
| `k_v_raw` | 1x1 | - | 原始速度修正系数 |
| `risk_raw_valid` | 1x1 | boolean | 原始风险是否有效 |
| `risk_source_id` | 1x1 | uint8 | 0=规则回退，1=Transformer |

### 7.8 RiskBus

来源：`Risk Supervisor`

消费者：`Lateral Controller Variant`、`Speed Outer Loop`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `r_low` | 1x1 | - | 平滑后综合风险 |
| `r_ey` | 1x1 | - | 平滑后横向误差风险 |
| `r_stab` | 1x1 | - | 平滑后稳定性风险 |
| `k_v` | 1x1 | - | 平滑后速度修正系数 |
| `risk_valid` | 1x1 | boolean | 风险输出是否可用 |
| `degraded_mode` | 1x1 | boolean | 是否已进入降级模式 |

### 7.9 MPCParamBus

来源：`Risk Supervisor`

消费者：`Lateral Controller Variant`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `q_y` | 1x1 | - | 横向误差权重 |
| `q_psi` | 1x1 | - | 航向误差权重 |
| `q_beta` | 1x1 | - | 侧偏角权重 |
| `q_r` | 1x1 | - | 横摆角速度权重 |
| `r_delta` | 1x1 | - | 转角惩罚权重 |
| `r_d_delta` | 1x1 | - | 转角变化率惩罚权重 |
| `beta_max` | 1x1 | rad | 侧偏角约束上限 |
| `yaw_rate_max` | 1x1 | rad/s | 横摆角速度约束上限 |
| `ay_max` | 1x1 | m/s^2 | 横向加速度约束上限 |
| `delta_max` | 1x1 | rad | 转角幅值约束上限 |
| `delta_rate_max` | 1x1 | rad/s | 转角变化率约束上限 |
| `v_ref_adapt` | 1x1 | m/s | 修正后的参考速度 |
| `param_valid` | 1x1 | boolean | 参数是否有效 |

### 7.10 CmdBus

来源：`Lateral Controller Variant` + `Speed Outer Loop`

消费者：`CarSim Interface`

| 字段 | 维度 | 单位 | 说明 |
|---|---:|---|---|
| `delta_cmd` | 1x1 | rad | 横向控制转角命令 |
| `ax_cmd` | 1x1 | m/s^2 | 纵向速度外环加速度命令 |
| `cmd_valid` | 1x1 | boolean | 控制命令是否有效 |
| `hold_last_cmd` | 1x1 | boolean | 是否保持上一时刻命令 |
| `control_mode_id` | 1x1 | uint8 | 0=PID, 1=Fixed MPC, 2=Rule-risk MPC, 3=Transformer-MPC |

### 7.11 建议的状态码

为了避免字符串进入闭环，建议统一使用如下状态码：

- `control_mode_id`
  - 0 = PID
  - 1 = Fixed MPC
  - 2 = Rule-risk MPC
  - 3 = Transformer-MPC

- `risk_source_id`
  - 0 = Rule fallback
  - 1 = Transformer

- `hold_last_cmd`
  - `true` = 保持上一时刻控制量
  - `false` = 正常刷新控制量

## 8. 在线输出与记录

### 8.1 设计原则

- 只记录在线闭环真正需要的信号
- 每个仿真 case 输出一份完整日志
- 原始测量、风险输出、MPC 参数、控制命令分层记录
- 所有日志必须按时间严格对齐
- 便于后处理生成 SCI 论文中的图和表

### 8.2 必须记录的在线信号

#### 基础信息

| 字段 | 说明 |
|---|---|
| `time` | 仿真时间 |
| `scenario_id` | 场景编号或名称 |
| `control_mode_id` | 当前控制器模式 |
| `sample_time` | 当前控制周期 |

#### 车辆状态与测量

| 字段 | 说明 |
|---|---|
| `x` | 车辆全局坐标 x |
| `y` | 车辆全局坐标 y |
| `yaw` | 车辆航向角 |
| `vx` | 纵向速度 |
| `vy` | 横向速度 |
| `yaw_rate` | 横摆角速度 |
| `beta` | 质心侧偏角 |
| `ay` | 横向加速度 |
| `delta_meas` | 实际前轮转角 |
| `ax_meas` | 实际纵向加速度 |
| `delta_rate_meas` | 实际转角变化率 |

#### 轨迹误差与参考量

| 字段 | 说明 |
|---|---|
| `e_y` | 横向误差 |
| `e_psi` | 航向误差 |
| `e_y_rate` | 横向误差变化率 |
| `e_psi_rate` | 航向误差变化率 |
| `kappa_ref` | 参考曲率 |
| `v_ref_base` | 基础参考速度 |
| `v_ref_adapt` | 风险修正后的参考速度 |
| `path_idx` | 当前参考点索引 |

#### Transformer 与风险输出

| 字段 | 说明 |
|---|---|
| `r_low_raw` | Transformer 原始综合风险 |
| `r_ey_raw` | Transformer 原始横向误差风险 |
| `r_stab_raw` | Transformer 原始稳定性风险 |
| `k_v_raw` | Transformer 原始速度修正系数 |
| `r_low` | 平滑后的综合风险 |
| `r_ey` | 平滑后的横向误差风险 |
| `r_stab` | 平滑后的稳定性风险 |
| `k_v` | 平滑后的速度修正系数 |
| `risk_source_id` | 风险来源标识 |
| `degraded_mode` | 是否进入降级模式 |
| `transformer_valid` | Transformer 输出是否有效 |

#### MPC 参数与求解状态

| 字段 | 说明 |
|---|---|
| `q_y` | 横向误差权重 |
| `q_psi` | 航向误差权重 |
| `q_beta` | 侧偏角权重 |
| `q_r` | 横摆角速度权重 |
| `r_delta` | 转角惩罚权重 |
| `r_d_delta` | 转角变化率惩罚权重 |
| `beta_max` | 侧偏角约束上限 |
| `yaw_rate_max` | 横摆角速度约束上限 |
| `ay_max` | 横向加速度约束上限 |
| `delta_max` | 转角幅值约束上限 |
| `delta_rate_max` | 转角变化率约束上限 |
| `mpc_feasible` | MPC 是否可行 |
| `mpc_status_code` | MPC 求解状态码 |
| `solve_time_ms` | MPC 求解时间 |

#### 控制输出

| 字段 | 说明 |
|---|---|
| `delta_cmd` | 横向控制命令 |
| `ax_cmd` | 纵向速度外环命令 |
| `cmd_valid` | 控制命令是否有效 |
| `hold_last_cmd` | 是否保持上一时刻命令 |

### 8.3 建议的日志文件组织

- `sim_log.csv`：逐时刻在线信号主日志
- `sim_log.mat`：Simulink 原始信号保存文件
- `case_meta.json`：场景参数、控制模式、采样周期、模型版本
- `diagnostic.mat`：可选调试信号，如窗口快照、Transformer 中间量

### 8.4 论文主图建议

#### 主图 1：轨迹跟踪效果图

内容：

- 车辆实际轨迹
- 参考轨迹
- 车道边界
- 低附着区间阴影

用途：

- 说明控制器是否真正跟上了轨迹

#### 主图 2：误差与稳定性响应图

内容：

- `e_y`
- `e_psi`
- `beta`
- `yaw_rate`
- `ay`

用途：

- 说明低附着工况下稳定性是否改善

#### 主图 3：Transformer 输出与优化参数联动图

内容：

- `r_low / r_ey / r_stab / k_v`
- `q_beta / q_r / beta_max / yaw_rate_max / v_ref_adapt`

用途：

- 直接证明 Transformer 输出参与了 MPC 参数调整

这张图是最关键的“参与优化证据”图。  
如果风险先变化，而 MPC 权重、约束和速度随后同步变化，就能看出 Transformer 不是独立展示，而是在驱动控制策略改变。

#### 主图 4：控制命令与求解状态图

内容：

- `delta_cmd`
- `ax_cmd`
- `mpc_feasible`
- `solve_time_ms`

用途：

- 说明控制量是否平滑
- 说明求解是否实时可行

#### 主图 5：方法对比图

内容：

- `PID`
- `Fixed MPC`
- `Rule-risk MPC`
- `Transformer-MPC`

用途：

- 证明 Transformer-MPC 的增量收益

### 8.5 补充图建议

#### 补充图 1：Transformer 风险输出示意图

内容：

- `r_low`
- `r_ey`
- `r_stab`
- `k_v`

用途：

- 单独展示 Transformer 的风险判断结果

#### 补充图 2：特征时间敏感性图

内容：

- feature-time saliency heatmap
- 或 attention heatmap

用途：

- 作为解释性补充图
- 不作为主证据图

#### 补充图 3：局部放大图

内容：

- 低附着切换前后 2 到 3 秒的局部窗口
- 风险输出
- MPC 参数变化
- 控制命令变化

用途：

- 更清楚地展示“风险触发 -> 参数调整 -> 控制响应”的链路

### 8.6 建议的论文证据链

建议论文中按以下顺序组织图像证据：

1. 先看风险输出是否提前变化
2. 再看 MPC 参数是否随风险联动
3. 再看车辆轨迹和误差是否改善
4. 最后用对比方法和消融实验证明增量价值

最能说明“Transformer 确实参与优化”的组合是：

- 风险输出曲线
- MPC 参数曲线
- 轨迹/误差改善曲线
- 与 `Fixed MPC`、`Rule-risk MPC` 的对比图

## 9. Transformer Risk Prediction

### 9.1 模块定位

该模块只做在线风险推理，不做控制求解，也不做离线训练。

职责：

- 接收固定长度历史窗口
- 执行特征标准化
- 调用训练好的 Transformer 风险模型
- 输出原始风险值和有效性标志

不做的事情：

- 不生成训练标签
- 不更新模型参数
- 不直接输出转角或加速度
- 不做风险到 MPC 参数映射

### 9.2 模块输入输出

#### 输入

| 字段 | 维度 | 类型 | 说明 |
|---|---:|---|---|
| `window` | `L x 14` | `double` | 原始历史特征窗口 |
| `window_ready` | 1x1 | `boolean` | 窗口是否填满 |
| `transformer_enable` | 1x1 | `boolean` | 当前周期是否执行推理 |
| `reset` | 1x1 | `boolean` | 仿真复位信号 |

#### 输出

| 字段 | 维度 | 类型 | 说明 |
|---|---:|---|---|
| `r_low_raw` | 1x1 | `double` | 原始综合风险 |
| `r_ey_raw` | 1x1 | `double` | 原始横向误差风险 |
| `r_stab_raw` | 1x1 | `double` | 原始稳定性风险 |
| `k_v_raw` | 1x1 | `double` | 原始速度修正系数 |
| `risk_raw_valid` | 1x1 | `boolean` | 原始输出是否可用 |
| `transformer_status_code` | 1x1 | `uint8` | 0=idle, 1=running, 2=ok, 3=fallback |
| `transformer_latency_ms` | 1x1 | `double` | 单次推理耗时 |

### 9.3 输入窗口规则

- `window` 的列顺序必须与第 7.6 节完全一致
- `L` 初版固定为 `16`
- 窗口不足时用零初始化或首样本填充，但必须置 `window_ready=false`
- `window_ready=false` 时不允许 Transformer 输出作为主控制依据

### 9.4 特征标准化

Transformer 在线推理前必须做标准化：

```text
x_norm = (x - mean_train) / std_train
```

说明：

- `mean_train` 和 `std_train` 作为只读模型附件保存
- 其顺序必须与第 7.6 节的特征顺序一致
- 标准化只作用于输入特征，不作用于输出风险

### 9.5 模型输出约束

Transformer 的输出头必须保持与 Python 版本一致的有界形式：

- `r_low_raw ∈ [0, 1]`
- `r_ey_raw ∈ [0, 1]`
- `r_stab_raw ∈ [0, 1]`
- `k_v_raw ∈ [k_v_min, 1]`

建议在网络尾部保留显式边界约束，在线侧再增加一次轻量限幅，防止数值异常。

### 9.6 模块内部结构

建议按以下顺序搭建为一个子系统：

1. `Window Input`
2. `Window Ready Check`
3. `Feature Normalize`
4. `Transformer Inference`
5. `Output Clamp`
6. `Validity & Status`

对应关系如下：

| 子块 | 职责 |
|---|---|
| `Window Input` | 接收 `HistoryBus.window` |
| `Window Ready Check` | 判断窗口是否可用 |
| `Feature Normalize` | 应用训练期均值和标准差 |
| `Transformer Inference` | 调用训练好的网络 |
| `Output Clamp` | 对输出做轻量限幅 |
| `Validity & Status` | 给出有效性和状态码 |

### 9.7 推理频率

Transformer 不必每个 MPC 周期都运行。

建议节拍：

- `Ts_mpc`：MPC 控制周期
- `Ts_tr`：Transformer 推理周期

建议初值：

- `Ts_mpc = 0.02 s ~ 0.05 s`
- `Ts_tr = 0.05 s ~ 0.10 s`

即 Transformer 低频运行，MPC 高频运行。  
两次 Transformer 推理之间，`Risk Supervisor` 可保持上一时刻有效风险值。

### 9.8 失败与回退

Transformer 结果无效的典型情形：

- `window_ready=false`
- 输入出现 `NaN` 或 `Inf`
- 模型加载失败
- 推理异常超时
- 输出超出合理边界

处理原则：

- `Transformer Risk Prediction` 只负责标记 `risk_raw_valid=false`
- 不在该模块内直接改写 MPC 参数
- 风险回退由 `Risk Supervisor` 统一处理

推荐回退逻辑：

1. Transformer 正常输出时，`risk_source_id=1`
2. Transformer 失效时，`risk_source_id=0`
3. `Risk Supervisor` 使用规则风险或上一时刻安全值

### 9.9 与 Python 原型的对应关系

在线模块对应 Python 版本中的：

- `transformer_model.py`
- `dataset.py` 中保存下来的标准化器参数

不对应在线模块的部分：

- `train_transformer.py`
- `risk_labels.py`

### 9.10 论文可视化直接关联

该模块最直接支持的图有：

- `r_low / r_ey / r_stab / k_v` 时间曲线
- 风险切换前后的局部放大图
- 风险输出与 MPC 参数联动图

这三类图是证明 Transformer 参与控制优化的核心证据。

## 10. Risk Supervisor

### 10.1 模块定位

Risk Supervisor 是 Transformer 与 MPC 之间的安全闸门和参数翻译层。

职责：

- 接收原始风险输出
- 做限幅、平滑和一致性检查
- 根据有效性决定是否回退
- 将风险映射为 MPC 参数和速度修正量

它是在线闭环中最关键的“防黑箱”模块。

### 10.2 模块输入输出

#### 输入

| 字段 | 维度 | 类型 | 说明 |
|---|---:|---|---|
| `r_low_raw` | 1x1 | `double` | Transformer 原始综合风险 |
| `r_ey_raw` | 1x1 | `double` | Transformer 原始横向误差风险 |
| `r_stab_raw` | 1x1 | `double` | Transformer 原始稳定性风险 |
| `k_v_raw` | 1x1 | `double` | Transformer 原始速度修正系数 |
| `risk_raw_valid` | 1x1 | `boolean` | Transformer 原始输出是否可用 |
| `transformer_status_code` | 1x1 | `uint8` | Transformer 状态码 |
| `v_ref_base` | 1x1 | `double` | 基础参考速度 |
| `reset` | 1x1 | `boolean` | 仿真复位信号 |

#### 输出

| 字段 | 维度 | 类型 | 说明 |
|---|---:|---|---|
| `r_low` | 1x1 | `double` | 平滑后的综合风险 |
| `r_ey` | 1x1 | `double` | 平滑后的横向误差风险 |
| `r_stab` | 1x1 | `double` | 平滑后的稳定性风险 |
| `k_v` | 1x1 | `double` | 平滑后的速度修正系数 |
| `risk_valid` | 1x1 | `boolean` | 风险结果是否可用 |
| `degraded_mode` | 1x1 | `boolean` | 是否进入降级模式 |
| `risk_source_id` | 1x1 | `uint8` | 0=规则回退, 1=Transformer |
| `MPCParamBus` | 1x1 | bus | 风险映射后的 MPC 参数 |
| `v_ref_adapt` | 1x1 | `double` | 修正后的参考速度 |

### 10.3 处理顺序

建议按以下顺序实现：

1. `reset` 时清空内部状态
2. 检查 `risk_raw_valid`
3. 对原始风险做限幅
4. 对风险做一阶平滑
5. 判断是否进入降级模式
6. 将风险映射为 MPC 参数
7. 计算 `v_ref_adapt`
8. 输出给横向 MPC 和速度外环

### 10.4 风险有效性规则

风险输入无效的典型条件：

- `risk_raw_valid=false`
- `transformer_status_code` 异常
- 风险值为 `NaN` 或 `Inf`
- 风险超出物理边界

处理原则：

- 若 Transformer 有效，则使用 Transformer 风险
- 若 Transformer 无效但规则风险可算，则使用规则风险
- 若两者都不可用，则保持上一时刻安全风险
- 若连续失效达到设定阈值，则退化到固定参数 MPC 口径

### 10.5 平滑与限幅规则

先做硬限幅，再做软平滑。

硬限幅：

```text
r_low  ∈ [0, 1]
r_ey   ∈ [0, 1]
r_stab ∈ [0, 1]
k_v    ∈ [k_v_min, 1]
```

软平滑采用一阶滤波：

```text
r_filt(k) = rho * r_filt(k-1) + (1 - rho) * r_raw(k)
```

其中：

- 风险上升时使用较快响应
- 风险下降时使用较慢回落

建议保留两个系数：

- `rho_up`
- `rho_down`

### 10.6 风险到 MPC 参数映射

建议映射规则与 Python 原型保持一致，便于论文与代码一致。

#### 权重映射

```text
q_y      = clip(q_y0      * (1 + 1.2 * r_ey),   q_y0,      q_y_max)
q_psi    = clip(q_psi0    * (1 + 1.0 * r_ey),   q_psi0,    q_psi_max)
q_beta   = clip(q_beta0   * (1 + 2.4 * r_stab), q_beta0,   q_beta_max)
q_r      = clip(q_r0      * (1 + 2.0 * r_stab), q_r0,      q_r_max)
r_delta  = clip(r_delta0  * (1 + 1.2 * r_stab), r_delta0,  r_delta_max)
r_d_delta= clip(r_d_delta0* (1 + 2.0 * r_stab), r_d_delta0,r_d_delta_max)
```

#### 约束映射

```text
beta_max      = clip(beta_max0      * (1 - c_beta * r_stab),     beta_min,      beta_max0)
yaw_rate_max  = clip(yaw_rate_max0  * (1 - c_yaw_rate * r_stab), yaw_rate_min,  yaw_rate_max0)
ay_max        = clip(ay_max0        * (1 - c_ay * r_stab),       ay_min,        ay_max0)
delta_rate_max= clip(delta_rate_max0* (1 - 0.45 * r_stab),       delta_rate_min,delta_rate_max0)
delta_max     = delta_max0
```

#### 速度修正

```text
speed_scale = min(k_v, 1 - c_v * r_low)
v_ref_adapt = v_ref_base * clip(speed_scale, k_v_min, 1)
```

### 10.7 内部状态建议

建议在 Risk Supervisor 内保留以下状态：

- `last_good_risk`
- `last_good_params`
- `fallback_counter`
- `recovery_counter`
- `supervisor_mode_id`

推荐状态机：

- `0 = normal`
- `1 = fallback`
- `2 = recovering`

### 10.8 与控制器的接口关系

Risk Supervisor 的输出直接供给：

- `Lateral Controller Variant`
- `Speed Outer Loop`
- `CarSim Interface & Logger`

其中：

- 横向 MPC 读取 `MPCParamBus`
- 速度外环读取 `v_ref_adapt`
- Logger 记录 `risk_source_id`、`degraded_mode`、`param_valid`

### 10.9 与 Python 原型的对应关系

在线模块对应 Python 版本中的：

- `risk_mapper.py`
- `controllers.py` 中的规则风险和 Transformer 风险调度逻辑

### 10.10 论文表达建议

这一层在论文里最好表述为：

> 采用显式风险监督层对 Transformer 输出进行边界约束、平滑滤波与可行性保护，并将风险因子映射为 MPC 权重、约束和参考速度修正量，从而保证深度学习模块只作为可解释的决策辅助而非直接控制器。

## 11. 在线闭环运行顺序

建议 Simulink 顶层按以下顺序组织：

1. `Reference & Scenario Manager` 初始化参考轨迹和场景
2. `CarSim Plant` 输出测量状态
3. `Error & Feature Builder` 计算轨迹误差和单步特征
4. `History Buffer` 更新历史窗口
5. `Transformer Risk Prediction` 在低频节拍下输出原始风险
6. `Risk Supervisor` 完成限幅、平滑、回退和参数映射
7. `Lateral Controller Variant` 求解横向 MPC 并输出 `delta_cmd`
8. `Speed Outer Loop` 输出 `ax_cmd`
9. `CarSim Interface` 接收控制命令并驱动车辆
10. `Logger` 同步记录所有在线信号

### 11.1 运行闭环中的关键边界

- `Transformer` 不直接连接到 MPC 求解器
- `Risk Supervisor` 是唯一的参数翻译入口
- `Speed Outer Loop` 不进入横向 MPC 代价函数
- `mu` 只用于场景设置，不作为在线输入

### 11.2 存档版本说明

本文档当前定义的是：

- MATLAB/Simulink + CarSim 在线闭环工程
- 联合控制结构
- 横向 MPC + 纵向速度外环
- 单模态时序风险感知
- 风险监督型参数映射

不包含：

- 数据集生成
- Transformer 训练
- 离线结果分析
- 论文作图脚本
