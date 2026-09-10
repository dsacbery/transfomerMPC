# Day 7 横向 MPC 与离线 Transformer 工具设计

## 目标

在 Day 2-6 冻结接口之上，交付固定参数横向 MPC 的可测试 MATLAB 实现，以及与 `16x14` 历史窗口契约一致的离线 Transformer 数据集、训练和导出脚本。Day 7 不接入在线 Transformer 推理、Risk Supervisor、速度外环或 CarSim 模型。

## 架构与接口

- `tmpsim.math.discretizeBicycleModel` 使用现有 `vehicle_params` 和 `tmpsim_config`，以 `vx_used=max(vx,1)` 建立四状态线性单轨模型并离散化，返回固定尺寸状态、输入、曲率前馈和横向加速度关系矩阵。
- `tmpsim.LateralMpcSystem` 保存上一有效转角，`step(err, meas, ref, mpcParam, resetRequested)` 返回 `delta_cmd`、`mpc_feasible`、`mpc_status_code`、`solve_time_ms`、`delta_ff` 和 `hold_last_cmd`。控制状态固定为 `[e_y,e_psi,beta,r]`，使用 `Np=15/Nc=5` 与 `mpcParam` 的 Q/R 和约束；输入非法或 QP 不可行时保持上一命令，并按冻结状态码报告。
- Python 工具只处理离线日志：数据集按场景划分，窗口列顺序严格为 `vx, vy, yaw_rate, ay, beta, delta_meas, delta_rate_meas, ax_meas, e_y, e_psi, e_y_rate, e_psi_rate, kappa_ref, v_ref_base`；训练/导出产物由调用方指定路径，不写入仓库。

## 验证

先新增 MATLAB 单元测试并确认缺失实现导致红灯；随后以最小实现通过直线、圆弧、前馈、动态参数、转角/速率约束、非法输入和不可行回退测试。最终运行 Day 2-7 全量 MATLAB 回归、Python 脚本自测、`checkcode` 与 `git diff --check`，并恢复模型测试产生的 `tmpsim_online.slx` 字节内容。
