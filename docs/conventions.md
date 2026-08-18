# 坐标、姿态与单位 / Coordinates, Attitude and Units

- 内核统一使用 SI：m、m/s、kg、s、N、N·m、rad。界面按字段标注显示 km、deg、rpm、kPa 等工程单位。
- `q_BI` 为标量在前的主动四元数 `[w,x,y,z]`，把机体系向量旋转到 ECI 惯性系。
- 姿态误差使用最短旋转四元数；CSV 相邻参考四元数先按点积调整符号，再进行 SLERP。
- 角速度保存在机体系。矩阵允许完整 3×3 惯量；当前用户界面输入主对角项。
- 轨道根数为半长轴、偏心率、倾角、升交点赤经、近地点幅角和真近点角；角度界面输入为度。
- 火箭轨迹计算在 ECI 中完成。3D 显示使用由发射点天向、下程向和横向构成的局部显示基，不反馈到动力学。
- 旋转和平动均使用固定步长 RK4。界面播放速度只决定每帧执行多少个固定物理步，不改变积分步长。

The dynamics core uses SI units. `q_BI` is a scalar-first active body-to-ECI quaternion. Display coordinates and playback never alter the physical state or integration step.

