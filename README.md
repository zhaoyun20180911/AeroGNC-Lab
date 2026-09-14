# AeroSys Lab v4.1｜航天系统综合仿真平台

## v4.1 重大更新（2026-09-14）

本次版本是项目的一次重大功能与产品升级：

- 软件正式升级为 **AeroSys Lab v4.1｜航天系统综合仿真平台**，顶层功能统一为“卫星姿轨控”“火箭上升入轨”“火箭一级回收”三个模块。
- 新增完整的火箭一级 RTLS 回收链路，包括翻转、返场点火、滑行、再入点火、栅格舵气动下降、动态着陆点火和滚动时域凸 QP 着陆制导。
- 回收默认交接状态更新为约 **80 km 分离高度**，并同步调整下场距离、速度、质量、推进剂、等效发动机组推力和阶段参数；六组标准扰动工况均通过安全着陆测试。
- 全软件重新设计为白底深色文字界面，顶部提供“中文 / English”开关；按钮、参数、曲线、指标、提示和设置弹窗均按当前模式单语显示。
- 保留并扩展卫星姿轨控与火箭上升入轨功能，完整 Release 回归测试全部通过。

## 下载 v4.1（Windows x64）

### 方式一：直接下载单文件 EXE（推荐）

这是绝大多数用户应选择的版本。无需安装 Visual Studio、CMake 或 Microsoft Visual C++ 运行库，下载后直接双击即可运行：

**[下载 AeroSys Lab v4.1 单文件 EXE](https://github.com/zhaoyun20180911/AeroGNC-Lab/releases/download/v4.1.0/AeroSys_Lab_v4.1_Windows_x64.exe)**

### 方式二：下载本地免安装部署包

适合需要在本地保存完整说明、校验文件或复制到其他 Windows 计算机的用户。ZIP 内含同一版本的 EXE、运行说明和 SHA-256 校验文件：

**[下载 AeroSys Lab v4.1 Portable ZIP](https://github.com/zhaoyun20180911/AeroGNC-Lab/releases/download/v4.1.0/AeroSys_Lab_v4.1_Windows_x64_Portable.zip)**

两种下载包含相同的软件功能，均为 Windows 10/11 x64 免安装版本。单文件 EXE 最方便；Portable ZIP 更适合归档、校验和本地分发。需要查看或修改源代码的开发者可使用 GitHub 的“Code → Download ZIP”或克隆本仓库。

AeroSys Lab 是使用 C++20 与原生 Win32/GDI 构建的 GNC 与动力学仿真平台。v4.1 采用白底深色文字界面，顶部“中文 / English”开关可在中文单语与英文单语之间即时切换，所有按钮、参数、下拉项、曲线、指标、提示和设置弹窗随模式同步刷新。软件包含三个彼此并列的顶层模块：

- **卫星姿轨控**：四类姿态任务、PD/PID/LQR、反作用飞轮、轨道保持、摄动与扰动仿真；
- **火箭上升入轨**：六组只读标称任务、闭环轨迹制导、两级动力学、TVC/RCS 和入轨指标；
- **火箭一级回收**：从一级分离状态开始的 RTLS 返回发射场闭环仿真。

## 直接运行

双击根目录中的 `运行 AeroSysLab.bat`。脚本会启动现有的 `build/Release/AeroSysLab.exe`；若文件不存在，则先配置并编译 Release 版本。

也可在 PowerShell 中执行：

```powershell
cmake -S . -B build -A x64
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
./build/Release/AeroSysLab.exe
```

依赖 Windows 10/11、CMake 3.24+、Visual Studio 2022 C++ 工具链。Release 使用静态 MSVC 运行库。

## 火箭一级回收工作流

启动软件后选择“火箭一级回收”（英文模式为“First-Stage Recovery”）。默认配置从约 80 km 一级分离状态开始，可直接运行完整链路：

```text
分离 → 翻转 → 返场点火 → 滑行 → 再入点火
     → 栅格舵气动下降 → 在线凸优化着陆 → TOUCHDOWN
```

默认分离状态为着陆场 NED 坐标下北向 20 km、高度 80 km、北向速度 600 m/s、向下速度 −150 m/s（仍在上升），一级总质量 90 t、可用回收推进剂 62 t，并采用 2.28 MN 等效发动机组。页面实时显示当前阶段、实际轨迹、目标着陆点、预测落点、位置/速度、质量与推进剂、推力、TVC、四片栅格舵和 QP 求解性能。下方参数区可直接修改分离 NED 状态、着陆场、车辆、风/大气、制导频率、动态点火余量和优化器参数；“恢复默认”可返回可直接演示的标称 RTLS 工况。

回收模块使用 ECEF 保存全局位置/速度，以着陆点 NED 进行局部制导，以四元数传播姿态。Landing 阶段采用滚动时域在线凸 QP，每个制导周期只执行当前解的第一个控制量，随后利用新状态重新建模求解。求解异常或超时会切换安全启发式控制并保留状态记录。

## 数据与测试

- 右上角 **导出 CSV** 可导出当前模块遥测；回收 CSV 包含阶段、ECEF/NED 状态、预测落点、执行机构和优化器字段。
- `gnc_tests` 覆盖原卫星与上升入轨模块；`recovery_tests` 覆盖标称、位置误差、速度误差、推力偏差、大气/风、组合扰动六组工况。
- 六组回收工况均须经历完整阶段序列并通过安全着陆门槛，测试同时检查坐标变换、分离状态交接、栅格舵修正、执行机构限幅、求解周期和 CSV。

运行全部测试：

```powershell
ctest --test-dir build -C Release --output-on-failure
```

## 工程说明

- [软件架构](docs/architecture.md)
- [卫星模型](docs/satellite_model.md)
- [火箭上升与任务数据](docs/rocket_model.md)
- [控制、轨控与扰动](docs/control_and_disturbance.md)
- [一级回收 V1 实现报告](AeroSys_Lab_Recovery_V1_Implementation_Report.md)

一级回收 V1 是 Falcon-9-like 教学与工程仿真，不是 SpaceX 飞控复刻，也不使用或声称使用真实私有参数。当前版本采用真值导航、参数化气动、等效发动机组和局部平动近似，接口已为后续传感器/EKF、高保真环境、查表气动和外部 QP 求解器预留。

许可证见 [LICENSE](LICENSE)。
