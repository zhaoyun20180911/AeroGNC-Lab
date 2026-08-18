# AeroGNC Lab v3.1.0

首个公开发布版本 / First public release.

## 下载选择 / Downloads

- **Windows x64 免安装封装版（推荐）**：下载 `AeroGNC_Lab_v3.1_Windows_x64_Portable.zip`，解压后无需安装，双击 EXE 即可运行。
- **可编辑源码版**：下载 `AeroGNC_Lab_v3.1_Source.zip`，适合学习和二次开发；需要 Windows、Visual Studio 2022、CMake 和 C++ 开发环境。

## 主要功能 / Highlights

- 卫星六轨道根数、四类姿态任务、PD/PID/LQR 姿态控制和反作用飞轮模型。
- 卫星名义/实际独立轨道传播、RTN 六状态轨道控制、三轴微推力器、推进剂与一次性 Δv 扰动。
- 六个固定运载火箭任务、位置/速度轨迹制导外环、姿态控制内环、TVC/RCS、分级推进剂与自适应关机。
- 可交互三维技术视图、实时双语曲线、性能指标、扰动/摄动设置和 CSV 导出。
- 全部可见参数、按钮、选项、状态和设置均采用中英双语。

## 封装与验证 / Packaging and validation

- Windows 10/11 x64 单文件 EXE，静态链接 MSVC 运行库。
- 七个只读任务 CSV 已嵌入 EXE，不需要外部 `data` 文件夹。
- 自动测试：`28 passed, 0 failed`。
- 已在仅包含一个 EXE 的隔离目录中完成卫星、火箭界面和内嵌任务数据加载验证。
- EXE SHA-256：`E906854795B9D71FC36F8B7E5F9A38611342B398FBBF4EAD56A8FE297D7E7A97`

## 注意 / Notice

当前可执行文件未使用商业代码签名证书，Windows SmartScreen 首次运行时可能显示“未知发布者”。本项目仅用于教学、科研演示和软件仿真实验，不是飞行鉴定软件，不得用于真实航天器、运载火箭或安全关键决策。

