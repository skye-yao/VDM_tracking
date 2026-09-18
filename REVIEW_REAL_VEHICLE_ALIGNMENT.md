# Python 与实车压缩包核查（2026-09-17）

## 结论

Python 四个 student 控制器的核心公式均已填写；现有 9 项单元测试全部通过。不能把 README 中“学生待填写版”的旧描述当成实际代码状态。但实验评测、结果留存和实车迁移验收尚未全部完成。本次仅核查，不修改 Python 或 C++，不修改原压缩包，等待用户判断这些缺口是否先处理。

组员提供的 MD 和压缩包内文档作为需求背景和待核对资料，不作为额外操作授权。

## 文件对应关系

| Python | 压缩包 | 关系 |
|---|---|---|
| vdm_lab/student/pure_pursuit.py | VDM课程_待修改补充代码/pp_tracking_controller student.cpp | 同属 PP，但前视距离公式、目标点距离处理和纵向接口不同 |
| vdm_lab/student/lqr_kinematic.py | VDM课程_待修改补充代码/lqr_tracking_controller student.cpp | Python 为四维横向误差状态、单转向输入；C++ 为三维位置/航向误差、两维输入，不能直接复制矩阵和权重 |
| vdm_lab/student/lqr_dynamic.py | 未在课程控制器目录中找到对应模板 | Python 的扩展算法，不是此次 C++ LQR 模板使用的模型 |
| vdm_lab/student/mpc.py | 未在课程控制器目录中找到对应模板 | 若要求三算法实车比较，需要另加控制器、求解器依赖与工程接入 |
| common、config、run_experiment.py | sydl_smartcar_ws.zip 内 ROS 工程 | Python 负责仿真与评价；ROS 工程负责状态、路径、控制调度和车辆接口，无需逐文件翻译 |

外层课程目录也含 pp_tracking_controller.cpp 和 lqr_tracking_controller.cpp 两个参考实现，学生版本的缺口为 PP-1～PP-5、LQR-1～LQR-6。完整车端工程包含相应头文件、车辆模型和控制器实现。sensors_ws.zip 是另一个工作空间，本次未深入审查其驱动。压缩包内车辆说明 DOCX 未进行逐页核查。

## Python 尚未收尾的部分

1. **组员实现与默认实验版本尚未对齐。** scripts/run_ablation.py:28 默认 solution，组员补写的是 student。MD 所给直接运行批量脚本的命令不会自动验证 student。调用时需要显式选择版本；这不是算法漏填，但影响实验能否证明组员的代码有效。
2. **异常运行的持久化记录不完整。** scripts/run_ablation.py:71-80 把失败保存在内存列表并打印；默认首次异常就退出且不生成总表。summarize_ablation.py:22-30 仅收集同时存在元数据和指标的运行，程序异常退出的组合不会自动成为失败行。正常结束但未到终点的运行则有 reached_goal 字段，两者需区别。
3. **终点指标与仿真配置没有统一。** common/simulation.py:193 用配置中的 stop_distance/stop_speed 判断结束，common/logging.py:136 却固定为 1.5 m/0.5 m/s。默认配置一致，但调整阈值后，成功/完成时间可能被错误标注。
4. **27 组 student 消融没有完整交付证据。** 本次看到的历史 integration-check 结果为 PP/MPC/两种 LQR 的 S 弯和圆形案例，不能替代 MD 所列三算法×三路径×三速度的矩阵。本次未补跑全矩阵。
5. **MPC 的 30 Hz 实时性尚未验证通过。** 本次在 D:/python/python.exe、cvxpy 1.9.2 下，student MPC、S 弯、dt=1/30、max_time=2，运行 60 步：平均计算 41.63 ms，P95 47.40 ms，最大 358.63 ms。统计包含首次调用开销，仅为本机短测，不能推断目标车载 C++ 性能。30 Hz 周期为 33.33 ms。每次求解重建 CVXPY 问题，求解失败直接抛异常；仿真实现已有，实车超时/失败处置尚需设计。

## 组员 MD 的准确程度

- PP/LQR/MPC 仿真框架、统一限幅、控制耗时与实车日志模板确实存在。
- “本机缺少 cvxpy”已不符合本次使用的 Python 环境；MPC 求解相关测试通过。
- “27 组最终比较尚未完成”不能被现有少量历史结果推翻。
- 实车日志文件是模板，不代表已自动采集或已有真实车辆数据。

## C++ 迁移前发现的具体问题

在内层 sydl_smartcar_ws/src/driverless_package/driverless/ 工程中：

- src/tracking_controller/lqr_tracking_controller.cpp 把参考前轮角转换为 deg 后送入 vehicle_model_->update()；src/vehicle_model/vehicle_model_bicycle_kinematic.cpp 直接对 ref_delta 调用 tan/cos，没有转换回 rad。
- 同一 LQR 控制器把航向误差转换为 deg，但模型 A、B 使用弧度线性化形式；result_.YawErr 也写入该度数，与外层接口文档的 rad 约定冲突。
- LQR 的 Q 初始化为第三行 [0, QValue_, 3]，第二行为 [0, QValue_, 0]；默认 QValue_=3 时矩阵不对称，不能当作正确的标准 LQR 权重直接沿用。
- PP 参考版只判断路径指针非空，没有同时验证路径非空、状态非空和最近点下标；末点情况下前视点循环可能一次不执行，距离/航向未按末点重新计算。

这些属于现成 C++ 参考工程的问题，不是 Python 漏填。后续补学生 C++ 时需要一并明确单位与模型，不能把参考版直接当作已经验证的正确实现。

## 本次验证范围

- python -m unittest discover -s tests -v：9/9 通过。
- student MPC 30 Hz 两秒仿真：运行成功，耗时数据如上。
- 静态阅读 student 控制器、测试、仿真循环、日志/批量汇总，以及压缩包课程代码和内层相关模型/控制器。
- 未运行 ROS 编译、车辆回放或实车测试。
