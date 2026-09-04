# Transformer-MPC Simulink 项目

这是 MATLAB/Simulink + CarSim 联合仿真项目的 Git 仓库骨架。

项目目标：

- 在线采集 CarSim 车辆状态
- 使用 Transformer 进行时序风险推理
- 通过 Risk Supervisor 将风险映射为 MPC 参数
- 使用横向 MPC 和纵向速度外环完成闭环控制
- 记录可复现的仿真日志和场景元数据

## 当前状态

当前仓库先建立目录、文档和接口占位，实际 MATLAB/Simulink 模型将在 Windows 开发环境中逐步加入。

空的 .slx、.sldd 和 .mat 文件不会作为占位文件提交；真实文件生成后再放入对应目录。

## 目录

- docs/：设计文档和仓库管理说明
- model/：Simulink 顶层模型和数据字典
- src/：MATLAB System、MATLAB Function 和工具函数
- config/：车辆、采样、MPC、风险和场景配置
- models/：Transformer 模型附件
- tests/：单元测试、模型测试和故障注入测试
- scripts/：初始化、单场景运行和回归脚本
- cases/：可复现的场景配置
- examples/：脱敏的小型示例
- logs/、results/、data/：运行生成目录，默认不上传实际结果

## 设计文档

- docs/Transformer_MPC_Simulink_Online_Design.md
- docs/Transformer_MPC_Simulink_Online_Implementation_Blueprint.md
- docs/Git_Private_Repository_Upload_Guide.md

## 开发环境

目标开发环境为 Windows MATLAB/Simulink + CarSim。每次模型变更应记录 MATLAB、Simulink、CarSim 版本，并在提交前运行相应测试。

## 首次运行

真实 MATLAB 文件加入后，预计使用以下入口：

    run('scripts/setup_tmpsim.m');
    result = run_case('cases/nominal_lane_change.json');

具体命令以 scripts/ 和 docs/ 中的最新说明为准。

## 仓库可见性

本仓库默认按 Private Repository 管理。CarSim 文件、Transformer 权重、车辆参数和实验数据上传前必须确认授权范围，不得提交 License、密钥、个人绝对路径或完整原始数据集。
