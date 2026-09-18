# PP / LQR 已补充代码

Python 的核心算法足以作为补写依据。本次完成压缩包现有 PP、LQR 学生模板，保留车端框架的车辆模型、接口与纵向规划，不是把 Python 四维 LQR 原样翻译成 C++ 三维 LQR。未新增 MPC。

## 在哪里查看

本目录两个 `.cpp` 可直接打开、复制或作为完整文件替换。

同级 `实车实验_已补充PP_LQR.zip` 是原始完整压缩包的修改副本：

- `实车实验/VDM课程_待修改补充代码/` 的两个 `student.cpp` 已填好；两个无 student 后缀的参考文件也同步为此版本，避免误用旧单位实现。
- `实车实验/实车-完整代码/sydl_smartcar_ws.zip` 内实际编译使用的两个控制器源文件也已替换。
- 原微信目录里的 `实车实验.zip` 不变。

## 替换已有工程

将本目录两个 cpp 覆盖到以下目录，保持文件名：

`sydl_smartcar_ws/src/driverless_package/driverless/src/tracking_controller/`

不需要复制 student 文件到编译目录，不要同时编译两份同名类的实现。不需要改头文件、车辆模型源文件或 CMakeLists.txt。

在原配套 ROS 环境中重新编译：

```bash
cd ~/sydl_smartcar_ws
catkin_make --pkg driverless
source devel/setup.bash
```

完整压缩包原有 build/devel 是旧产物，不包含此次修改，不能替代重新编译。跨机器迁移宜在干净的 catkin 工作空间中使用 src 并重新构建。

选择算法使用原 `src/smartcar/launch/driverless.launch` 中的 `tracking_controller_class`，值为 `PP` 或 `LQR`。

## 已完成

- PP：有符号横向误差、归一化航向误差、速度/误差相关前视距离、目标点搜索、实际距离几何转角、末点及零距离保护。
- LQR：三维位置/航向误差、曲率前馈、模型更新、Riccati 迭代、LDLT 线性求解、负反馈和机械限角。
- LQR 内部 yaw、参考转角、转角增量全部为 rad；最终 SteerAngle 转为 deg。模型源码直接使用 tan/cos，因此不再将度数传给模型。
- Q 保留学生模板正确的对角矩阵，避免旧参考版本非对称 Q。
- 两个控制器输出 Speed 为 km/h，SteerAngle 为 deg，Decel 为 m/s²，LateralErr 为 m，YawErr 为 rad；YawErr 统一为参考航向减当前航向。LQR 内部误差仍为当前减参考。
- 空指针、空路径、无效索引、非有限输入、求解数值异常时清空旧结果，输出零速度和车辆配置的最大减速度。
- 保留原曲率限速、当前转角限速及制动函数；PP 保留课程模板的最低驱动速度处理。

## 实现边界与验证

- LQR 静止线性化采用最低 0.5 m/s，避免零速时控制矩阵退化；纵向制动仍使用实际速度。
- Riccati 达到次数上限但数值有限时使用有限次迭代增益，并每 5 秒最多警告一次；这不代表已经满足收敛精度。权重、迭代次数和车辆标定参数仍需在目标环境验证。
- 本次核对了原工程头文件、模型、几何工具与调用接口；数值核查脚本覆盖 PP 转向方向/末点、角度跨界、模型 Jacobian 和 LQR 反馈稳定性。
- 数值核查运行的是 Python 对公式的独立复核，**不是 C++ 编译测试**。当前环境没有配套 ROS/catkin；本次未完成 ROS 编译、回放或实车验收，交付为待目标环境编译验证的源代码。
- 原车端停车、避障、模式切换、控制频率和转向速率执行仍由原框架负责；本次没有补齐整个实车日志链路，也没有声称具备完整安全系统验收。
