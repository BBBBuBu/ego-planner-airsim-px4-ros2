# ego-planner-airsim-px4-ros2

UE4 / AirSim / PX4 SITL 中的 EGO-Planner ROS 2 避障成功基线。

仓库按本机工程的 **hw-ros2/ros2/src/** 层级组织，便于逐文件对照。原工程未修改；此次整理只移动归档路径并更新说明，成功基线源码内容保持不变。

## 文件夹对应关系

```text
hw-ros2/
├── ros2/
│   └── src/
│       ├── ego-planner-swarm/
│       │   ├── LICENSE
│       │   ├── UPSTREAM_README.md
│       │   └── src/
│       │       ├── planner/
│       │       │   ├── plan_manage/       # FSM、轨迹交接、launch、RViz
│       │       │   ├── plan_env/          # 深度投影与占据地图
│       │       │   ├── bspline_opt/       # B 样条优化
│       │       │   ├── path_searching/    # 搜索与规划预算
│       │       │   ├── traj_utils/        # 轨迹消息与可视化
│       │       │   ├── drone_detect/      # 上游辅助模块
│       │       │   └── rosmsg_tcp_bridge/ # 上游辅助模块
│       │       └── uav_simulator/Utils/quadrotor_msgs/
│       ├── hw_insight/hw_insight/
│       │   ├── position_cmd_to_px4.py
│       │   └── ego_tracking_diagnostics.py
│       ├── airsim_ros_pkgs/
│       ├── airsim_interfaces/
│       └── px4_msgs/
├── config/airsim/settings.json
├── baseline/                         # 参数、上游来源、源码校验清单
├── docs/                             # 运行与成功记录说明
├── LICENSES/                         # 保留各组件的许可证
└── tools/verify_snapshot.py
```

例如，本机 `/home/airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp` 对应仓库中的 `hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp`。

## 核心文件

| 功能 | 仓库路径 |
| --- | --- |
| 重规划状态机 | [ego_replan_fsm.cpp](hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp) |
| 轨迹调度与恢复握手 | [traj_server.cpp](hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/traj_server.cpp) |
| 规划管理 | [planner_manager.cpp](hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/planner_manager.cpp) |
| 深度与地图 | [grid_map.cpp](hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp) |
| 优化器 | [bspline_optimizer.cpp](hw-ros2/ros2/src/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp) |
| PX4 控制桥 | [position_cmd_to_px4.py](hw-ros2/ros2/src/hw_insight/hw_insight/position_cmd_to_px4.py) |
| 仿真入口 | [single_run_in_sim.launch.py](hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py) |
| 地图与规划参数 | [advanced_param.launch.py](hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.launch.py) |
| AirSim 配置 | [settings.json](hw-ros2/config/airsim/settings.json) |

`hw_insight` 只归档上述两条直接运行的脚本，未包含教学示例和完整教学包元数据。`quadrotor_msgs` 放回原工程所在的嵌套位置，colcon 可递归识别。

## 成功基线与环境

2026-11-11 本地测试中，用户观察到无碰壁、无障碍碰撞并成功到达目标；完整日志中最后轨迹 68 后进入 WAIT_TARGET。起步曾有交接重试和停稳恢复，之后任务完成。

- ROS 2 Humble，Ubuntu / WSL2，UE4 + AirSim，PX4 工程标记 v1.15.2。
- 目标 ENU `(0.15,15.0,2.0)` m，起飞高度 2 m。
- 规划速度上限 0.5 m/s，加速度上限 1.0 m/s²。
- **当前成功基线障碍膨胀为 0.5 m**，地图分辨率 0.1 m。
- 相机 200×150、FOV 90°，AirSim 相机安装 Z=1 m，与 grid_map.cpp 外参配套。
- 桥采用单调控制定时器、`px4_timestamp_mode=receiver`，进程内累计恢复上限 3 次。

[参数与来源清单](hw-ros2/baseline/source_manifest.json)及[成功记录](hw-ros2/docs/success_baseline.md)保留本轮验证范围。原标签 `baseline-2026-11-11` 保留最初目录快照，main 使用本机工程层级。

## 构建核心规划包

```bash
git clone https://github.com/BBBBuBu/ego-planner-airsim-px4-ros2.git
cd ego-planner-airsim-px4-ros2/hw-ros2/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ego_planner px4_msgs \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

需要 Eigen、PCL、OpenCV、cv_bridge、pcl_conversions、message_filters 等依赖。初次归档已在独立目录验证 7 个核心包构建成功；本次目录移动没有改动这些源码。

AirSim ROS 包还依赖完整 AirSim SDK，见 [AirSim 接入说明](hw-ros2/docs/airsim_integration.md)。UE4 场景资产、完整 PX4 固件和 AirSim SDK 单独准备。

## 运行顺序

每个 ROS 终端 source 对应工作区的 install/setup.bash，只运行一套 planner/server/bridge。

1. 在 UE4 启动配套 AirSim 场景。
2. 在配套 PX4 工程运行 `make px4_sitl_default none_iris`。
3. 按固件网络配置启动 Micro XRCE-DDS Agent，例如 `MicroXRCEAgent udp4 -p 8888`。
4. 启动 `ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=<Windows主机IP>`。
5. 启动 `ros2 launch ego_planner rviz.launch.py`。
6. 启动 `ros2 launch ego_planner single_run_in_sim.launch.py`。
7. 在本仓库 `hw-ros2/ros2/` 目录下启动桥，等待起飞稳定：

```bash
/usr/bin/python3 src/hw_insight/hw_insight/position_cmd_to_px4.py \
  --ros-args -p mode:=ego -p fixed_height:=2.0 \
  -p px4_timestamp_mode:=receiver
```

8. 发送触发：

```bash
ros2 topic pub --once /traj_start_trigger geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}'
```

## 归档核验

在仓库根目录运行：

```bash
python3 hw-ros2/tools/verify_snapshot.py
```

源码、配置和许可证按 SHA-256 校验。日志、缓存、build、install、录包和编辑器文件通过 `.gitignore` 排除。

来源于 [ZJU FAST Lab EGO-Planner ROS 2 分支](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)。各组件保留自身许可证，见[第三方说明](hw-ros2/LICENSES/README.md)。
