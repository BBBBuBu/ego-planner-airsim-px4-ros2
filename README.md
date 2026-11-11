# ego-planner-airsim-px4-ros2

EGO-Planner ROS 2 在 UE4 / AirSim / PX4 SITL 中的避障集成与成功基线归档。

使用 UE4 场景和 AirSim 深度相机生成局部障碍地图，由 EGO-Planner 输出轨迹，经 ROS 2 桥转换为 PX4 OFFBOARD 设定点。基于 [ZJU FAST Lab 的 ros2_version 分支](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)。

## 基线结果

2026-11-11 的本地 UE4 场景测试中，用户观察到飞行期间没有碰壁或碰撞障碍，并成功到达设定目标。完整启动日志在最后一条轨迹 68 后进入 WAIT_TARGET。起步时仍有轨迹交接重试和一次停稳恢复，随后任务完成；本基线没有宣称日志完全没有告警。

这是已验证场景的代码快照。UE4 地图资产、PX4 固件源码、完整 AirSim/AirLib 和编译产物不在本仓库。克隆后需要配置相应仿真环境。

## 目录

```text
src/ego-planner-swarm/  修改后的规划器、地图、优化器、轨迹交接与 launch
src/quadrotor_msgs/    EGO 消息依赖
src/px4_msgs/          本机成功版本使用的 PX4 ROS 2 消息定义
bridge/               PositionCommand → PX4 桥与只读跟踪采集
config/airsim/         本次使用的 settings.json
airsim_overlay/        当前 AirSim ROS 包和接口的源码覆盖层
baseline/             参数、来源版本及逐文件 SHA-256
```

AirSim 覆盖层带 COLCON_IGNORE；构建方式见 [AirSim 接入](docs/airsim_overlay.md)。原工程的 ROS 包路径是 AirSim 源码根目录下的 ros2/src，因此该覆盖层不作为独立 AirLib SDK 发布。

## 环境与参数

- ROS 2 Humble，Ubuntu / WSL2，UE4 + AirSim。
- PX4 本地工程标记 v1.15.2；Git 基准与未校验的本地修改范围记录在 baseline/source_manifest.json。
- 单机 PX4，规划坐标 world / ENU，PX4 接收 NED。
- 目标 `(0.15, 15.0, 2.0)` m，起飞高度 2 m。
- 最大规划速度 0.5 m/s，最大加速度 1.0 m/s²。
- **本基线的障碍膨胀为 0.5 m**，地图分辨率 0.1 m。
- 相机 200×150、FOV 90°；AirSim 安装 Z=1 m，与 grid_map.cpp 中对应的机体外参配套。
- 桥控制定时器使用单调时钟，px4_timestamp_mode=receiver；恢复上限为进程内累计 3 次。

## 构建规划器

先安装 ROS 2 Humble 开发工具以及 Eigen、PCL、OpenCV、cv_bridge、pcl_conversions、message_filters 等依赖。工作区包含的 px4_msgs 必须与实际运行固件消息匹配。

```bash
source /opt/ros/humble/setup.bash
cd ~/ego-planner-airsim-px4-ros2
colcon build --packages-up-to ego_planner px4_msgs \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

该命令构建 AirSim 接入所需核心包；未收录原工程的随机森林、独立动力学仿真器与模型资源，其他上游 launch 不属于此精简快照的验证入口。

## 运行顺序

每个 ROS 终端 source 当前工作区 setup.bash；AirSim ROS 覆盖层工作区需先构建并 source。WSL2 下先确认 Windows 与 Ubuntu 时间稳定，避免两套校时服务互相调整系统时钟。

1. 在 UE4 中启动已配置的 AirSim 地图。
2. 在配套 PX4 工程运行 `make px4_sitl_default none_iris`。
3. 启动 Micro XRCE-DDS Agent，保持与 PX4 SITL 使用的 UDP 端口一致（常用命令 `MicroXRCEAgent udp4 -p 8888`，以固件配置为准）。
4. 启动 `ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=<Windows主机IP>`。
5. 启动 `ros2 launch ego_planner rviz.launch.py`。
6. 启动规划器并保存控制台输出：

```bash
ros2 launch ego_planner single_run_in_sim.launch.py 2>&1 | tee ~/ego_arrival_diag.log
```

7. 启动桥，等待起飞并稳定悬停：

```bash
cd ~/ego-planner-airsim-px4-ros2
/usr/bin/python3 bridge/position_cmd_to_px4.py \
  --ros-args -p mode:=ego -p fixed_height:=2.0 \
  -p px4_timestamp_mode:=receiver
```

8. 发送一次任务触发：

```bash
ros2 topic pub --once /traj_start_trigger geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}'
```

仅运行一套 planner/server/bridge。固定高度参数用于起飞悬停，不把后续 EGO 三维轨迹的高度强制覆盖为常数。

## 关键话题

| 话题 | 用途 |
| --- | --- |
| /airsim_node/PX4/CameraDepth/DepthPlanar | 浮点深度图 |
| /airsim_node/PX4/odom_local_ned | AirSim 里程计 |
| /grid_map/odom_enu | 规划器使用的 ENU 里程计 |
| /drone_0_planning/pos_cmd | EGO 位置、速度、加速度参考 |
| /fmu/in/trajectory_setpoint | 发给 PX4 的 NED 设定点 |
| /drone_0_planning/safety_hold | 永久停止锁存信号 |
| /drone_0_planning/recovery_* | 停稳后重新规划与接管握手 |

## 核验与来源

运行 `python3 scripts/verify_snapshot.py` 校验导出源码与快照清单一致。

本机归档验证结果与成功记录见 [基线说明](docs/success_baseline.md)。来源和许可证见 [第三方说明](LICENSES.md)。UE4 资产和完整 AirSim/PX4 环境需要单独准备。
