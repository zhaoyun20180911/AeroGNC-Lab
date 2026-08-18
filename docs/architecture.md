# 软件架构 / Software Architecture

```text
Win32 bilingual GUI / 双语界面
├─ scenario, parameters, validation / 场景、参数与校验
├─ control, disturbance, perturbation dialogs / 三类设置弹窗
├─ mouse-interactive 3D technical view / 鼠标交互 3D 技术视图
├─ four selectable real-time plots / 四张可选实时图
└─ metrics and CSV export / 指标与 CSV 导出
                         │ SimulationSample only
Simulation layer / 仿真层
├─ SatelliteSimulation
│  ├─ classical elements → ECI orbit
│  ├─ four attitude reference generators
│  └─ reaction wheels + rigid-body dynamics
└─ RocketMissionSimulation
   ├─ immutable mission repository + interpolation
   ├─ nominal reference and independent truth state
   ├─ staging, thrust, atmosphere, drag and TVC/RCS
   ├─ insertion snapshot and phase-aware tracking metrics
   └─ independent nominal/actual two-body coast
                         │
Shared core / 公共核心
├─ Vec3, Mat3, Quaternion, SLERP and RK4
├─ PD, PID and LQR controller manager
├─ deterministic composable disturbances
└─ orbital-element conversions
```

图形层只读取 `SimulationSample` 和配置，不包含控制或动力学公式。任务 CSV 由 `MissionRepository` 只读加载，实际仿真状态从首个标称 ECI 状态初始化，之后独立传播。

The GUI consumes simulation samples and configuration only. Mission CSV files are immutable runtime inputs. The truth state is initialized from the first nominal ECI state and propagated independently thereafter.
