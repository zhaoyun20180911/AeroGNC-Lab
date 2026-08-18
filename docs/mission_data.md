# 火箭任务数据 / Rocket Mission Data

运行时目录：`data/rocket_missions/`

| 发射场 / Site | 300 km · 28.5° | 500 km · 51.6° |
|---|---|---|
| 文昌 / Wenchang | `wenchang_300km_28p5deg.csv` | `wenchang_500km_51p6deg.csv` |
| 西昌 / Xichang | `xichang_300km_28p5deg.csv` | `xichang_500km_51p6deg.csv` |
| 卡纳维拉尔角 / Cape Canaveral | `cape_300km_28p5deg.csv` | `cape_500km_51p6deg.csv` |

`mission_summary.csv` 提供六个任务的发射场、目标轨道、两级质量/推力/Isp/气动参数、惯量、TVC 限制和末端摘要。

轨迹文件提供时间、ECI 位置/速度、地理位置、高度、下程/横程、质量、级段、推力、参考俯仰/方位、参考四元数/角速度、大气密度、动压和阻力。

七个文件均为用户提供的只读输入。构建系统只执行逐字节复制；加载器不写入、不规范化文件内容，也不在缺失时生成替代数据。

