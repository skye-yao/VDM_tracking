# R3 实车路径跟踪日志与分析模板

本模板用于把 PP、运动学 LQR、线性 MPC 的实车数据与仿真结果放进同一套分析框架。实验开始前由指导教师或车辆管理员确认车辆状态、场地和速度上限；本模板不替代车辆操作规程。

## 1. 实验编号与目录

一次实车运行使用一个唯一编号，推荐：

```text
YYYYMMDD_<path>_<algorithm>_<speed>_<run>
示例：20260916_lane_change_pp_5kph_01
```

建议每组保留以下文件：

```text
real_vehicle_logs/
  20260916_lane_change_pp_5kph_01/
    experiment.yaml       # 本模板第 2 节的元数据
    run.bag               # 原始 ROS 数据；如有权限且磁盘空间足够
    tracking.csv          # 统一后的逐周期日志
    reference_path.csv    # 实际录制、实际使用的参考路径
    summary.png           # 轨迹和时序汇总图
    notes.md              # 现场异常、人工接管和天气/场地说明
```

同一算法、路径、速度至少重复 3 次。不能删除失败或人工接管的运行；应标记失败原因。

## 2. 每次运行的元数据 `experiment.yaml`

```yaml
experiment_id: 20260916_lane_change_pp_5kph_01
date_local: 2026-09-16
operator: <姓名或代号>
observer: <姓名或代号>

algorithm: pp                 # pp | lqr_kinematic | mpc
algorithm_commit: <Git commit 或代码版本>
path_id: lane_change_01
path_source: recorded         # recorded | planned
path_repeat: false
target_speed_kph: 5.0
control_rate_hz: 30.0

vehicle_id: <车号>
vehicle_param_file: <实际加载的 YAML 绝对路径>
wheelbase_m: <现场确认值>
max_roadwheel_angle_deg: <现场确认值>
max_steering_speed_degps: <现场确认值>

odom_topic: /gps_odom
command_topic: /vehicleCmdSet
vehicle_state_topic: /vehicleStateSet
tracking_state_topic: /tracking_state

rtk_fixed_before_start: false
manual_takeover: false
emergency_stop: false
run_result: pending           # completed | aborted | failed | invalid
abort_or_failure_reason: ""
notes: ""
```

注意：话题名以当前车辆的 launch 文件和 `rostopic list` 为准；上面的名字是讲义中的默认约定。转向角、速度和轴距也必须从**当前实际加载**的车辆 YAML 确认，不能使用仿真默认值代替。

## 3. 上车前记录清单

| 项目 | 结果 | 备注 |
| --- | --- | --- |
| 场地、路径和速度计划已获确认 |  |  |
| 人工接管与急停可用 |  |  |
| GPS 与 RTK 状态满足课程/车辆要求 |  |  |
| 底盘、CAN、相机/雷达状态正常 |  |  |
| 参考路径已录制，起终点与行驶方向已复核 |  |  |
| 控制器版本、参数文件、单位已记录 |  |  |
| 磁盘空间足够保存 rosbag |  |  |
| 观察员、操作员与停止条件已明确 |  |  |

出现车辆状态异常、定位状态不满足要求、路径不清晰或现场人员要求停止时，本次运行标记为 `aborted` 或 `invalid`，保留日志而不计入精度排名。

## 4. 原始数据采集

在确认实际话题名后，可用 rosbag 保留原始证据：

```bash
rosbag record -O run.bag \
  /gps_odom \
  /vehicleCmdSet \
  /vehicleStateSet \
  /tracking_state
```

若车辆发布的话题不同，修改上面的话题名，并同步写入 `experiment.yaml`。至少应能够恢复以下四类量：

1. 实际位姿：`x`、`y`、`yaw`、时间戳；
2. 实际速度与实际前轮角；
3. 下发目标速度、目标前轮角、减速度；
4. 控制器计算的横向误差、航向误差与最近路径点编号。

## 5. 统一逐周期日志 `tracking.csv`

单位统一为 m、s、rad、m/s；如果原 ROS 消息是 km/h 或 deg，转换后写入本文件，同时在元数据中保留原始单位说明。

| 字段 | 单位 | 含义 |
| --- | ---: | --- |
| `time_s` | s | 相对本次运行开始的时间 |
| `x_m`, `y_m`, `yaw_rad` | m, m, rad | 实际车辆位姿 |
| `speed_mps` | m/s | 实际车速 |
| `target_speed_mps` | m/s | 控制器下发的目标速度 |
| `steer_rad` | rad | 实际或最终下发的前轮角，注明来源 |
| `target_steer_rad` | rad | 控制器原始目标前轮角 |
| `acceleration_mps2`, `deceleration_mps2` | m/s² | 下发纵向命令 |
| `lateral_error_m` | m | 带符号横向误差 |
| `heading_error_rad` | rad | 带符号航向误差 |
| `target_index` | - | 最近/目标路径点编号 |
| `reference_s_m` | m | 目标路径点累计里程 |
| `curvature_1pm` | 1/m | 参考点曲率 |
| `control_compute_ms` | ms | 控制器单周期计算时间；若未记录填空并说明 |
| `mode` | - | manual / autonomous / emergency 等实际状态 |

对齐规则：以控制命令的时间戳为主时间轴；将定位、车辆反馈用最近时间戳或插值对齐。必须记录对齐方法及最大允许时间差。

## 6. 单次运行结果表

| 字段 | 数值 | 说明 |
| --- | ---: | --- |
| `run_result` |  | completed / aborted / failed / invalid |
| `reached_goal` |  | true / false |
| 参考路线长度 |  m | `reference_path.csv` 最后一个 `s_m` |
| 有效完成用时 |  s | 仅 `reached_goal=true` 时填写 |
| 平均推进速度 | m/s | 路线长度 / 有效完成用时 |
| 平均绝对横向误差 MAE | m | `mean(abs(lateral_error))` |
| 横向误差 RMSE | m | `sqrt(mean(lateral_error^2))` |
| P95 横向误差 | m | `percentile(abs(lateral_error), 95)` |
| 最大绝对横向误差 | m | `max(abs(lateral_error))` |
| 终点误差 | m | 实际终点至参考终点距离 |
| 平均绝对转角变化率 | rad/s | `mean(abs(diff(steer) / diff(time)))` |
| 最大绝对转角变化率 | rad/s | `max(abs(diff(steer) / diff(time)))` |
| 控制计算平均 / P95 | ms | 计算开销；MPC 必填 |
| 最大偏差时刻与位置 | s, m | 同时记录路径里程与现场标记 |

## 7. 汇总消融表

同一张表只比较同一辆车、同一路径、同一速度和同一定位条件下的算法。若速度不同，按速度分组或明确标注。

| 路径 | 速度 | 算法 | 重复次数 | 到达率 | MAE / m | RMSE / m | P95 / m | 最大误差 / m | 完成用时 / s | P95 计算耗时 / ms | 备注 |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 直线 | 5 km/h | PP |  |  |  |  |  |  |  |  |  |
| 直线 | 5 km/h | LQR |  |  |  |  |  |  |  |  |  |
| 直线 | 5 km/h | MPC |  |  |  |  |  |  |  |  |  |
| 双移线 | 5 km/h | PP |  |  |  |  |  |  |  |  |  |
| 双移线 | 5 km/h | LQR |  |  |  |  |  |  |  |  |  |
| 双移线 | 5 km/h | MPC |  |  |  |  |  |  |  |  |  |
| S 弯/圆弧 | 10 km/h | PP |  |  |  |  |  |  |  |  |  |
| S 弯/圆弧 | 10 km/h | LQR |  |  |  |  |  |  |  |  |  |
| S 弯/圆弧 | 10 km/h | MPC |  |  |  |  |  |  |  |  |  |

完整的课程矩阵应覆盖 3 算法 × 3 路径 × 2 速度；表中仅列出填写格式。每个格子使用重复运行的均值，并在备注中说明失败次数和原因。

## 8. 必交可视化

每个代表性组合至少输出一张 `summary.png`，包含：

1. 参考轨迹、实际轨迹、最大偏差点和起终点；
2. 横向误差与航向误差随时间变化；
3. 实际/目标速度，以及实际/目标转角；
4. 曲率与最大偏差时刻的对应关系；
5. MPC 额外展示预测轨迹或求解耗时分布。

汇总图建议按“路径 × 速度”分面，颜色代表算法；不要把不同路径、不同速度的时间序列堆在同一张图中。

## 9. 仿真与实车对照结论模板

```text
工况：<路径>，<速度>，车辆参数文件 <文件>。

仿真中 <算法> 的 MAE / RMSE / P95 分别为 <数值>；实车中分别为 <数值>。
实车误差相对仿真的主要变化发生在 <起步/入弯/曲率切换/终点>。
当时实际车速为 <数值>，实际转角为 <数值>，定位/执行延迟证据为 <日志字段或图号>。

结论：<算法> 在该工况下的优点是 <...>，限制是 <...>。
下一次复测只修改 <一个参数或一个因素>，其余路径、车辆、速度和日志方式保持不变。
```

## 10. 上车前的最低准备状态

- 仿真已完成 3 算法 × 3 路径 × 3 速度，且保存 `ablation_summary.csv`；
- 实车首跑的 PP 参数、LQR 参数和目标低速已确定；
- 实际车辆 YAML、话题名、单位和坐标系已现场确认；
- 数据记录节点或 rosbag 命令已在不开启自动驾驶时验证可用；
- MPC 若计划上车，已验证其 P95 单步计算时间低于实际控制周期，并定义求解失败的安全处理方式；
- 操作人员、人工接管、急停和停止条件均已按车辆操作规程确认。
