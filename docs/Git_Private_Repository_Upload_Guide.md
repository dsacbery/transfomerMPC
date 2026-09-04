# MATLAB/Simulink + CarSim 项目 GitHub 私有仓库上传指南

## 1. 文档目的

本指南用于将 Transformer-MPC MATLAB/Simulink + CarSim 项目安全地整理并上传到 GitHub 私有仓库。

本指南只管理以下内容：

- MATLAB/Simulink 工程源文件
- CarSim 联合仿真接口和项目配置
- Transformer 在线推理所需的模型附件
- 测试、场景和运行脚本
- 设计文档与版本说明

本指南不要求现在立即执行 Git 初始化、远程仓库创建或代码推送。执行命令前，应先检查文件内容和授权范围。

## 2. 推荐仓库策略

### 2.1 仓库可见性

初版建议使用 **Private Repository**，原因是：

- CarSim 项目文件可能受软件许可限制
- Transformer 权重可能属于未公开研究成果
- 车辆参数、实验场景和论文数据可能需要保密
- 私有仓库更适合导师、组内成员和协作者共同开发

后续如果需要公开代码，应另外制作脱敏版仓库，不要直接将私有仓库切换为 Public。

### 2.2 推荐仓库名称

`transformer-mpc-simulink`

也可以使用带课题或实验室前缀的名称，但仓库名称应避免包含个人电脑路径、实验数据编号或敏感项目名称。

### 2.3 推荐分支

```text
main        可复现、可交付的稳定版本
develop     集成开发分支
feature/*   单个功能开发分支
experiment/* 参数或场景实验分支
```

建议为可复现节点打标签：

```text
v0.1.0-baseline-mpc
v0.2.0-transformer-risk
v0.3.0-online-closed-loop
```

## 3. 推荐目录结构

```text
transformer-mpc-simulink/
├── README.md
├── LICENSE
├── .gitignore
├── .gitattributes
├── docs/
│   ├── Transformer_MPC_Simulink_Online_Design.md
│   ├── Transformer_MPC_Simulink_Online_Implementation_Blueprint.md
│   └── Git_Private_Repository_Upload_Guide.md
├── model/
│   ├── tmpsim_online.slx
│   └── tmpsim_online.sldd
├── src/
│   └── +tmpsim/
├── config/
│   ├── tmpsim_config.m
│   ├── vehicle_params.m
│   ├── scenario_catalog.m
│   ├── feature_stats.mat
│   └── transformer_manifest.mat
├── models/
│   └── transformer_weights.mat
├── tests/
├── scripts/
├── cases/
├── examples/
└── logs/
```

其中：

- `docs/`：设计文档和使用说明
- `model/`：Simulink 顶层模型和数据字典
- `src/`：MATLAB System、MATLAB Function 和工具函数
- `config/`：车辆、采样、MPC、风险和场景配置
- `models/`：Transformer 推理模型或模型索引
- `tests/`：单元测试、模型测试和故障注入测试
- `scripts/`：初始化、单场景运行和回归脚本
- `cases/`：可复现的场景 JSON 文件
- `examples/`：经过脱敏的小型示例
- `logs/`：运行输出目录，不上传实际日志

当前工程已有的两份设计文件应纳入 `docs/`：

```text
Transformer_MPC_Simulink_Online_Design.md
Transformer_MPC_Simulink_Online_Implementation_Blueprint.md
```

## 4. 必须上传的文件

### 4.1 工程源代码

上传：

- `*.m`
- MATLAB System 类
- MATLAB Function 源代码
- 纯函数和工具函数
- 参数配置脚本
- 场景配置脚本

不要只上传 `.slx` 而不上传其依赖的 MATLAB 源代码和配置文件。

### 4.2 Simulink 文件

通常上传：

- 顶层模型 `*.slx`
- 子系统模型 `*.slx`
- 数据字典 `*.sldd`
- 模型初始化脚本
- Bus 和参数定义

不上传：

- `slprj/`
- `*_simulink_cache*`
- `*.slxc`
- 自动保存文件 `*.asv`
- 临时恢复文件

### 4.3 测试和运行脚本

必须上传：

- 单元测试
- 信号契约测试
- Transformer 推理测试
- Risk Supervisor 测试
- MPC 测试
- 闭环 smoke test
- 故障注入测试
- `setup_*.m`
- `run_case.m`
- `run_regression_suite.m`

### 4.4 配置和元数据

上传：

- MATLAB/Simulink/CarSim 版本要求
- 采样时间
- 车辆参数
- MPC 基线参数
- 风险映射参数
- 特征顺序
- 特征均值和标准差
- 场景 JSON
- 模型输入输出签名

配置文件中不得写入个人绝对路径，例如：

```text
/Users/某个人/...
C:\Users\某个人\...
```

应使用相对路径或通过 `setup_tmpsim.m` 自动定位项目根目录。

## 5. CarSim 文件处理规则

### 5.1 私有仓库中可以上传

在确认授权允许共享的前提下，可以上传项目专用的：

- 车辆参数文件
- 道路和场景文件
- 联合仿真配置文件
- CarSim 与 Simulink 接口配置
- 用于复现实验的项目文件

### 5.2 不应上传

禁止上传或谨慎处理：

- CarSim 软件安装目录
- CarSim 软件本体
- License 文件
- 官方完整数据库
- 无权再分发的车辆或轮胎数据
- 依赖本机安装路径的缓存文件

如果无法确认某个 CarSim 文件是否可以上传，默认不上传该文件，改为在 `docs/carsim_setup.md` 中记录：

```text
需要的 CarSim 版本
需要导入的项目文件名称
文件来源
本地放置路径
导入或绑定步骤
```

### 5.3 公共仓库处理

将私有仓库改为 Public 之前，应单独检查：

- CarSim 文件是否有再分发许可
- 车辆参数是否属于公开数据
- Transformer 权重是否可以公开
- 实验场景是否包含敏感信息
- 日志和结果是否包含个人或机构信息

## 6. Transformer 权重处理规则

### 6.1 可以直接上传的内容

如果模型允许共享且体积较小，可以上传：

- `transformer_manifest.mat`
- `feature_stats.mat`
- 网络权重文件
- 输入输出签名
- 模型版本说明

### 6.2 大文件处理

GitHub 单文件超过 100 MB 时不能按普通 Git 文件上传。对于较大的 `*.mat`、`*.slx`、`*.sldd` 或 CarSim 文件，使用 Git LFS：

```bash
git lfs install
git lfs track "models/*.mat"
git lfs track "model/*.slx"
git lfs track "model/*.sldd"
```

执行后必须检查 `.gitattributes` 是否被更新：

```bash
git diff -- .gitattributes
```

### 6.3 不公开权重时

如果 Transformer 权重不能公开：

- 私有仓库只上传受控版本
- 公开版本只上传 manifest 和下载说明
- 记录模型版本和 SHA-256 校验值
- 不把下载令牌、服务器密码或私有链接写入 Git

## 7. .gitignore 建议内容

建议在仓库根目录创建 `.gitignore`，至少包含：

```gitignore
# macOS / Windows
.DS_Store
Thumbs.db

# MATLAB / Simulink generated files
slprj/
*_simulink_cache*/
*.slxc
*.asv
*.autosave
*.m~
*.tmp

# Generated logs and results
logs/*
results/*
data/raw/*
data/generated/*
!logs/.gitkeep
!results/.gitkeep

# Build artifacts
*.mex*
*.dll
*.so
*.dylib

# Private credentials and local settings
.env
.env.*
*.key
*.pem
*.token
credentials.*

# Local machine files
private/
local/
```

不要使用过宽的规则：

`*.mat`

因为 `feature_stats.mat` 和 Transformer 模型附件可能是在线推理所必需的。应只忽略日志和结果目录中的 MAT 文件。

## 8. .gitattributes 建议内容

建议在仓库根目录创建 `.gitattributes`：

```gitattributes
*.m       text eol=lf
*.md      text eol=lf
*.json    text eol=lf
*.yaml    text eol=lf
*.yml     text eol=lf

*.slx     binary
*.sldd    binary
*.mat     binary
*.sim     binary
*.par     binary
*.rdf     binary
```

如果这些二进制文件使用 Git LFS，还应由 `git lfs track` 自动写入对应的 `filter=lfs`、`diff=lfs` 和 `merge=lfs` 配置，不要手工删除这些配置。

## 9. README 必须说明的内容

仓库根目录的 `README.md` 至少说明：

1. 项目目标：Transformer 风险监督型横向 MPC + 纵向速度外环。
2. 支持的 MATLAB、Simulink、CarSim 版本。
3. 需要的 MATLAB Toolbox。
4. CarSim 文件的来源和授权限制。
5. 如何运行 `setup_tmpsim.m`。
6. 如何运行 nominal case。
7. 如何运行单元测试和回归测试。
8. 模型权重是否随仓库提供。
9. 哪些目录是运行后生成的。
10. 当前稳定版本对应的 Git tag。

README 中不要写入：

- 私人电脑绝对路径
- License 内容
- GitHub Token
- 服务器密码
- 未公开实验结论

## 10. 首次初始化和上传流程

以下命令只在确认目录内容无敏感文件后执行。

### 10.1 检查当前目录

```bash
pwd
rg --files | sort
git status --short
```

如果 `git status` 提示当前目录不是 Git 仓库，不代表可以立即上传；先完成 `.gitignore`、目录整理和敏感文件检查。

### 10.2 初始化本地仓库

```bash
git init -b main
git status --short
```

如果当前目录已经是 Git 仓库，不要再次执行 `git init`，先检查：

```bash
git rev-parse --show-toplevel
git remote -v
```

### 10.3 添加文件前检查

```bash
git status --short --ignored
git check-ignore -v logs/example.mat
git grep -n -I -E 'token|password|passwd|api[_-]?key|secret|license' -- . ':!docs'
```

如果发现凭据、License 或个人绝对路径，应先移除或改为模板占位符，不要提交后再补救。

### 10.4 创建首次提交

推荐首次提交只包含工程骨架和文档：

```bash
git add README.md .gitignore .gitattributes docs config model src tests scripts cases
git status --short
git diff --cached --stat
git diff --cached --name-only
git commit -m "chore: initialize private MATLAB Simulink project"
```

如果当前还没有 `README.md`、`config/` 或 `src/`，应只添加实际存在且已经检查过的路径，不要为了凑目录创建空的未知文件。

### 10.5 绑定 GitHub 私有仓库

先在 GitHub 网页端创建空的 Private Repository，不要勾选自动生成 README、`.gitignore` 或 License，以避免首次合并冲突。

SSH 方式：

```bash
git remote add origin git@github.com:<GITHUB_USER>/transformer-mpc-simulink.git
git remote -v
```

HTTPS 方式：

```bash
git remote add origin https://github.com/<GITHUB_USER>/transformer-mpc-simulink.git
git remote -v
```

不要把访问 Token 直接写进远程 URL。

### 10.6 首次推送

```bash
git push -u origin main
```

推送完成后，在 GitHub 网页端检查：

- 仓库仍为 Private
- README 正常显示
- 两份设计文档存在
- `.gitignore` 和 `.gitattributes` 生效
- 没有上传日志、缓存、License 或密钥
- 大文件是否被正确识别为 Git LFS

## 11. 推荐提交顺序

为便于回溯，建议按以下顺序提交：

```text
Commit 1  工程骨架、README、设计文档和忽略规则
Commit 2  Bus、配置和车辆参数
Commit 3  CarSim 适配和参考轨迹模块
Commit 4  HistoryBuffer 和 Transformer 推理接口
Commit 5  Risk Supervisor 和横向 MPC
Commit 6  纵向速度外环、日志和故障处理
Commit 7  单元测试、闭环测试和回归场景
```

提交信息应说明一个完整变化，例如：

```text
feat: add transformer risk inference interface
feat: add risk supervisor fallback state machine
feat: add lateral MPC constraint mapping
test: add transformer fault injection case
docs: update CarSim setup instructions
```

不要使用大量无法说明内容的提交信息，例如：

```text
update
test
final
new
```

## 12. 协作规则

### 12.1 修改 .slx 或 .sldd

Simulink 模型和数据字典属于二进制文件，建议：

- 一次只由一个人修改同一个模型文件
- 修改前先拉取最新 `main` 或 `develop`
- 修改后记录 MATLAB/Simulink 版本
- 提交前运行模型检查和 smoke test
- 在提交信息中说明修改了哪些子系统

Git 对 `.slx` 和 `.sldd` 的逐行合并能力有限，不能像 `.m` 文件一样依赖自动合并。

### 12.2 参数实验

实验参数不要直接覆盖稳定基线。建议：

- 基线参数保留在 `config/`
- 实验参数放在 `experiment/*` 分支或单独的 case JSON
- 每次实验记录随机种子、场景 ID 和模型版本
- 通过 Git tag 固定可复现实验节点

### 12.3 Pull Request 检查

合并前至少确认：

- 单元测试通过
- smoke test 通过
- 没有新增绝对路径
- 没有新增密钥或 License 文件
- CarSim 文件授权范围未扩大
- Transformer 输入输出顺序没有变化
- Bus 字段没有未经说明的改名

## 13. 日志、结果和数据管理

### 13.1 默认不上传

以下内容不进入普通 Git 提交：

- 完整仿真日志
- 大规模原始数据集
- 自动生成的结果文件
- 临时调试数据
- 论文全部绘图中间文件

### 13.2 可以保留的小型示例

为了帮助协作者快速验证，可以在 `examples/` 中保留：

- 脱敏后的短时日志
- 低维测试输入
- 固定尺寸的 mock Transformer 输出
- 不含敏感信息的 nominal case

示例文件必须在 README 中说明用途，不能让协作者误以为它们是完整实验结果。

## 14. 上传前最终检查表

### 文件范围

- [ ] 两份设计 Markdown 已纳入 `docs/`
- [ ] MATLAB 源代码、Simulink 模型、Data Dictionary 和测试文件齐全
- [ ] CarSim 只上传允许共享的项目文件
- [ ] Transformer 权重已确认是否可以共享
- [ ] 日志、结果和缓存已被忽略

### 安全检查

- [ ] 没有 GitHub Token、密码、私钥或 API Key
- [ ] 没有 CarSim License
- [ ] 没有个人绝对路径
- [ ] 没有未经授权的官方数据库
- [ ] 没有需要保密的原始数据

### 可复现性检查

- [ ] README 写明 MATLAB/Simulink/CarSim 版本
- [ ] README 写明所需 Toolbox
- [ ] `setup_tmpsim.m` 能定位项目相对路径
- [ ] 场景 JSON 包含场景 ID、仿真时长、控制模式和随机种子
- [ ] 模型权重和标准化参数有版本说明
- [ ] nominal case 的运行命令已记录

### Git 检查

- [ ] `git diff --cached --stat` 内容符合预期
- [ ] `git diff --cached --name-only` 没有敏感文件
- [ ] `git status --short --ignored` 中缓存和日志被正确忽略
- [ ] 大文件已决定使用普通 Git 还是 Git LFS
- [ ] 首次提交后已在网页端确认仓库为 Private

## 15. 推荐的初始执行顺序

1. 在 GitHub 创建空的 Private Repository。
2. 在本地整理目录和文档。
3. 创建并检查 `.gitignore`、`.gitattributes` 和 `README.md`。
4. 检查 CarSim 文件和 Transformer 权重的授权范围。
5. 初始化本地 Git 仓库。
6. 先提交工程骨架和设计文档。
7. 再分阶段提交 MATLAB、Simulink、CarSim 接口和测试代码。
8. 完成 nominal smoke test 后打第一个版本标签。

第一阶段的目标是建立一个安全、可复现、能被协作者理解的私有仓库，而不是一次性上传所有历史数据和运行结果。

