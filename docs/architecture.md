# 软件架构 / Software Architecture

```text
AeroSys Lab v4.1 Win32 GUI
├─ 卫星姿轨控 / Satellite GNC
├─ 火箭上升入轨 / Launch to Orbit
├─ 火箭一级回收 / First-Stage Recovery
├─ 参数校验、四张实时曲线、交互视图
└─ 指标汇总与独立 CSV 导出
                         │ SimulationSample
Simulation layer
├─ SatelliteSimulation
├─ RocketMissionSimulation
└─ RecoverySimulation
   ├─ RecoveryNavigation (truth navigation V1)
   ├─ RecoveryMissionManager / state machine
   ├─ LandingPointPredictor
   ├─ phase guidance + LandingGuidance
   ├─ ConvexLandingProblem → QpSolverInterface
   ├─ RecoveryControlAllocator → Engine/TVC/RCS/Grid Fin
   ├─ GridFinAerodynamicModel
   └─ RecoveryMetrics / telemetry / handoff
                         │
Shared core
├─ Vec3, Mat3, Quaternion, SLERP
├─ rigid-body rotational RK4
├─ PD, PID, LQR and anti-windup
├─ atmosphere/disturbance utilities
└─ orbit and coordinate conversions
```

图形层只读取配置与 `SimulationSample`，不承担制导、控制或动力学计算。回收 Guidance 只读取 `RecoveryNavigation` 输出的估计状态，不访问 Dynamics 私有变量。

回收模块全局状态为 ECEF，着陆场局部制导为 North-East-Down，姿态状态始终为四元数。平动恢复模型在着陆场局部 NED 中积分，再映射为 ECEF 状态；姿态继续复用公共刚体 RK4。分离状态通过版本化文件接口导入/导出，避免模块间私有变量耦合。
