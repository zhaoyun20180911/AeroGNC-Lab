# 卫星模型 / Satellite Model

- 轨道由六个经典根数转换为 ECI 初值。实际轨道与未受扰名义轨道是两个独立状态，均在球形地球二体引力下用 RK4 传播，不使用状态吸附。
- 姿态任务包括：对地定向、惯性 Euler 目标、地面经纬度目标跟踪、指定时刻开始的姿态机动。
- 姿态采用 `q_BI` 和机体系角速度，控制器输出机体系期望力矩。
- 三个正交反作用飞轮分别执行力矩，并施加最大力矩和最大转速限制；飞轮惯量用于角动量/转速更新。
- 轨道控制器使用名义/实际 RTN 位置与速度误差形成六状态反馈，生成 ECI 修正加速度，并按当前姿态换算为本体系三轴微推力器组的合力指令。推力受合推力上限、死区和推进剂余量约束。
- 轨控推进剂质量属于卫星初始湿质量的一部分，按 `ṁ = T/(Isp·g₀)` 消耗；推进剂耗尽后轨控推力强制归零，累计 Δv 继续保留为结果量。
- 可组合外扰包括恒定力矩、有限时脉冲、正弦力矩、确定性随机力矩，以及在指定时刻施加的一次性 RTN/LVLH 速度脉冲。
- 摄动窗口中的 J2、大气阻力、月球/太阳三体引力和太阳光压为 3.1 界面占位项，全部标注“未启用 / Not enabled”，当前轨道后端仍严格采用二体模型。
- 三维视图从当前实际 ECI 位置/速度反算瞬时轨道根数，并用同一米到像素比例绘制实轨和地球；因此 LEO、高轨及不同偏心率会呈现正确的相对尺度与轨道形状。
- 三维视图同时绘制虚线名义轨道、实际瞬时轨道、名义/实际卫星位置、二者误差连线和轨控力矢量。

The satellite supports arbitrary valid classical elements, four attitude-reference generators, constrained reaction wheels, independent nominal/actual two-body propagation, six-state orbit feedback, a fuel-limited three-axis thruster cluster, and a one-shot RTN delta-v disturbance. Orbit-perturbation entries remain explicit UI placeholders and do not alter the two-body backend.
