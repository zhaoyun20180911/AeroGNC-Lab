# AeroSys Lab v4.1 一级回收 GNC V1 实现报告

## 1. 最终目录结构

```text
include/gnc/
├─ math.hpp, control.hpp, simulation.hpp
└─ recovery.hpp
src/recovery/
├─ recovery_coordinates.cpp
├─ recovery_defaults.cpp, recovery_cases.cpp
├─ recovery_mission.cpp, recovery_guidance.cpp
├─ recovery_landing_guidance.cpp, recovery_qp_solver.cpp
├─ recovery_actuators.cpp, recovery_grid_fin.cpp
├─ recovery_simulation.cpp
├─ recovery_telemetry.cpp, recovery_handoff.cpp
└─ recovery_labels.cpp
tests/recovery_test.cpp
```

## 2. 重要修改文件

- `CMakeLists.txt`：工程版本改为 4.1.0，输出改为 `AeroSysLab.exe`，加入回收库与测试目标。
- `src/main.cpp`、`include/gnc/localization.hpp`：增加“火箭一级回收”顶层模块、配置、运行、可视化、曲线、汇总和 CSV，并实现浅色主题与中文/英文单语切换。
- `include/gnc/simulation.hpp`：增加 Recovery 场景及共用回收遥测字段。
- `src/settings_dialogs.cpp`、`resources/AeroGNC.rc`：更新 AeroSys Lab v4.1 品牌和 Windows 文件版本。
- 启动、构建、截图与便携打包脚本：改用 `AeroSysLab.exe` 和 v4.1 产品名。

## 3. 新增模块

- **Mission Manager**：固定状态机 `INITIALIZE → FLIP → BOOSTBACK → COAST → ENTRY_BURN → AERO_DESCENT → LANDING → TOUCHDOWN`，异常进入 `FAILED`。
- **Navigation**：V1 真值导航接口，Guidance 只接收 `RecoveryEstimatedState`。
- **Flip**：平滑步进的四元数 SLERP 指令与 RCS 姿态闭环。
- **Boostback**：飞行中反复调用共享落点预测器，按预测落点误差在线修正推力方向与截止条件。
- **Entry**：逆速度方向的闭环速度/能量减速与阶段退出逻辑。
- **Grid Fin**：四片独立舵面、角度/速率限幅，以及接收 Mach、动压、迎角、侧滑角、舵偏的可替换参数化气动模型。
- **Landing**：动态制动距离触发、3DOF NED 预测、滚动时域在线凸 QP、热启动、超时/不可行/数值错误回退。
- **Solver**：`QpSolverInterface` 与无外部依赖的 box-constrained projected-coordinate 凸 QP 求解器。
- **Control Allocation**：按阶段分配等效发动机、TVC、RCS 与栅格舵；发动机包含点火/关机延迟、推力滞后、节流与推进剂消耗。
- **GUI / Metrics**：阶段、实际轨迹、目标/预测落点、四张曲线、12 项回收汇总指标和专用 CSV。
- **Handoff**：带 `AEROSYS_RECOVERY_STATE_V1` 标识的一级分离状态与着陆场文件导入/导出接口。

## 4. 复用模块

- 复用公共 `Vec3`、`Mat3`、`Quaternion`、SLERP、姿态误差与刚体 RK4。
- 复用既有 `AttitudeController` 的 PD/PID/LQR、执行机构限幅和 anti-windup 机制。
- 复用 `SimulationSample`、实时曲线、参数校验、CSV 和 Win32/GDI 框架。
- 原卫星和上升入轨传播器未复制或搬移；回收作为第三个并列仿真对象接入。

## 5. 实际算法流程

回收真值以 ECEF 对外保存，先转换到着陆点 NED。任务管理器选择阶段制导；翻转阶段生成连续四元数参考，返场和气动下降阶段共同使用弹道落点预测，返场采用在线预测—校正，气动段依据北/东落点误差给四片栅格舵指令。着陆点火由当前下降速度、质量和可用推力计算制动距离，而非固定高度。

默认交接状态采用着陆场 NED 坐标：北向位置 20 km、东向位置 0、高度 80 km，北向速度 600 m/s、东向速度 0、向下速度 −150 m/s（负号表示仍在上升）；一级总质量 90 t，其中回收推进剂 62 t，最大等效发动机组推力 2.28 MN、最低节流 13%。这些参数作为可闭环验证的 Falcon-9-like 教学工况，不代表任何真实型号的精确分离数据。

Landing 阶段状态为 `[rN,rE,rD,vN,vE,vD]`，控制为期望净加速度。目标函数包含末端位置、末端速度、控制消耗和控制平滑；约束包含最大/最小推力形成的竖向加速度盒约束与水平加速度限制。预测步长按剩余高度和制动时间在配置上限内缩短。每次只执行首个控制量，再用新状态热启动重求解，经期望推力矢量、姿态控制和控制分配转换到发动机/TVC。

## 6. V1 简化与求解器选择

- 这是 Falcon-9-like 教学模型，不是 Falcon 9 参数复刻。
- Recovery 平动采用着陆场局部平面 NED、常重力与指数大气，再映射为 ECEF；未加入地球自转、椭球重力和高保真六自由度耦合平动。
- Navigation 为真值导航，未加入 IMU/GNSS/雷达高度计误差和 EKF。
- 落点预测器为含紧凑阻力因子的快速弹道预测；栅格舵为参数化解析模型；发动机为等效发动机组。
- 未引入 OSQP。原因是本项目要求生成无需额外运行库的本地 Windows EXE；V1 使用稳定的内置凸 QP 求解器并严格保留 `QpSolverInterface`，未来可直接添加 OSQP adapter。
- Landing QP 使用凸盒约束近似推力方向和能力，没有实现全 6DOF SCvx、着陆腿接触或发动机多机离散选择。

## 7. 六组测试结果

Release、固定步长 0.02 s、制导 10 Hz。以下为 2026-09-14 最终自动测试结果：

| 工况 | 终态 | 落点误差 m | 水平速度 m/s | 垂直速度 m/s | 耗油 kg | QP 成功率 | 平均/最大耗时 ms |
|---|---|---:|---:|---:|---:|---:|---:|
| Nominal | TOUCHDOWN | <0.001 | <0.001 | 0.132 | 57738.8 | 87.74% | 0.765 / 3.338 |
| Position Error | TOUCHDOWN | <0.001 | <0.001 | 0.132 | 57742.7 | 85.28% | 0.771 / 3.308 |
| Velocity Error | TOUCHDOWN | <0.001 | <0.001 | 0.136 | 57747.5 | 84.43% | 0.764 / 3.513 |
| Thrust Bias +3% | TOUCHDOWN | <0.001 | <0.001 | 0.144 | 57947.3 | 86.49% | 0.774 / 3.385 |
| Atmosphere / Wind | TOUCHDOWN | 0.028 | <0.001 | 0.131 | 57739.2 | 84.73% | 0.769 / 3.377 |
| Combined（含 -3% 推力） | TOUCHDOWN | 0.023 | <0.001 | 0.118 | 57470.3 | 84.14% | 0.813 / 3.481 |

自动测试还验证：六阶段顺序完整、ECEF/NED 往返、分离状态文件往返、执行机构不越界、气动扰动下栅格舵启用且预测落点误差下降、QP 耗时低于 100 ms 制导周期、CSV 架构完整、安全着陆阈值同时满足。原 `gnc_tests` 29 项全部通过。

## 8. Known Issues

- 简化模型与理想真值导航使最终误差小于真实飞行可达到的量级，结果用于闭环与软件架构验证，不能解释为真实火箭性能预测。
- 分离状态导入/导出核心接口已实现并测试；本版 GUI 仍以可编辑 NED 字段作为主要自定义入口，尚未增加专用文件选择按钮或上升模块自动跳转。
- 现有主 GUI 是历史形成的 Win32 单文件视图控制器；回收算法已拆分，但未来可继续把三类视图从 `main.cpp` 解耦。
- 内置求解器只处理当前 V1 的 box-constrained convex QP；更一般线性约束应由未来 OSQP adapter 承担。

## 9. Build / Run

```powershell
cmake -S . -B build -A x64
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
./build/Release/AeroSysLab.exe
```

也可双击 `运行 AeroSysLab.bat`。进入“火箭一级回收”，保持默认参数并点击“开始”，即可完成一次 RTLS 演示。CSV 由右上角“导出”生成。

## 10. Final Status

| Milestone | 状态 | 说明 |
|---|---|---|
| 0 Software Rename | PASS | AeroSys Lab v4.1、窗口/资源/脚本/README/EXE 已更新 |
| 1 Recovery Architecture | PASS | 独立顶层模块与职责接口完成 |
| 2 Initial State + Dynamics | PASS | ECEF/NED、Truth Navigation、传播与交接接口完成 |
| 3 Flip | PASS | 平滑 SLERP、RCS、闭环与切换完成 |
| 4 Boostback | PASS | 共享落点预测、在线修正、Engine/TVC 完成 |
| 5 Coast + Entry | PASS | 滑行、触发、再入闭环与关机完成 |
| 6 Grid-Fin Descent | PASS | 四片栅格舵、气动接口、误差下降测试完成 |
| 7 Online Convex Landing | PASS | 动态点火、QP、热启动、回退、安全着陆完成 |
| 8 GUI / Visualization / Summary | PASS | 浅色独立页面、中文/英文单语切换、视图、曲线、参数、指标、CSV 完成 |
| 9 Robustness Testing | PASS | 六组标准工况全部安全 TOUCHDOWN |

最终状态：**AeroSys Lab v4.1 一级回收 GNC V1 完成，可本地构建运行。未执行 GitHub 上传。**
