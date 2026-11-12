# ego-planner-airsim-px4-ros2

UE4 / AirSim / PX4 SITL 中的 EGO-Planner ROS 2 避障工程与成功基线。

**从 `airsim/` 开始，按本机 `/home/airsim/` 下的工程层级归档运行核心。**本次增加 AirLib、MavLinkCom、Eigen/rpclib、DDS Agent、PX4 本地修改及构建入口。排除缓存、编译产物、日志、录包、备份、测试程序和教学示例。

## 文件夹层级

```text
airsim/
├── hw-ros2/
│   ├── AirLib/                    # 深度/状态 RPC 客户端 SDK 与 Eigen
│   ├── MavLinkCom/                # AirSim MAVLink 通信库
│   ├── external/rpclib/rpclib-2.3.0/
│   ├── cmake/                     # SDK 核心库构建规则，不含测试目标
│   ├── ros2/src/
│   │   ├── ego-planner-swarm/
│   │   │   └── src/
│   │   │       ├── planner/        # FSM、地图、搜索、优化、轨迹管理
│   │   │       └── uav_simulator/Utils/quadrotor_msgs/
│   │   ├── hw_insight/            # 可构建的最小 PX4 桥包
│   │   ├── airsim_ros_pkgs/
│   │   ├── airsim_interfaces/
│   │   └── px4_msgs/
│   ├── config/airsim/settings.json
│   ├── baseline/                 # 版本、参数和逐文件校验清单
│   ├── LICENSES/
│   ├── docs/
│   └── tools/verify_snapshot.py
├── px4v1.15.2/
│   ├── BASELINE_SOURCE.json       # PX4 v1.15.2 确切提交与本地覆盖文件
│   ├── src/modules/mavlink/streams/DISTANCE_SENSOR.hpp
│   ├── Tools/setup/ubuntu.sh
│   └── LICENSE
├── Micro-XRCE-DDS-Agent-v2.4.2/    # Agent 核心源码，保留本地 FastDDS 版本修正
├── tools/                        # 环境检查、固件准备、构建和 Agent 启动
└── docs/                         # 新机器部署说明
```

PX4 的完整上游源码和子模块通过准备脚本按锁定提交下载到 `airsim/px4v1.15.2/`，再保留上述本地修改；下载内容、上游测试和模型不提交到这个核心仓库。普通 `git clone` 不会自动完成此步骤。

## 新机器使用步骤

前提：Ubuntu 22.04 / ROS 2 Humble 开发环境，以及 Windows UE4 + 兼容 AirSim 插件和场景。UE4 场景资产、引擎、插件二进制与 GPU 驱动不在源码仓库内。

```bash
git clone https://github.com/BBBBuBu/ego-planner-airsim-px4-ros2.git
cd ego-planner-airsim-px4-ros2
bash airsim/tools/check_environment.sh
python3 airsim/tools/fetch_px4.py
```

若 WSL 的代理地址不可用，可对公开依赖下载使用 `python3 airsim/tools/fetch_px4.py --direct`。它固定 PX4 为 v1.15.2、提交 `4817c0618a1286846116e90c6eb8919efaa013cf`，保留本地修改，并检查已有工作区，遇到不匹配时不会 reset 或覆盖。

安装构建依赖后依次运行：

```bash
bash airsim/tools/build.sh agent
bash airsim/tools/build.sh ros
bash airsim/tools/build.sh px4
```

上述是构建操作，不启动飞行或自动解锁。Agent 的第三方依赖构建可能需要联网。ROS 构建关闭 BUILD_TESTING；本仓库不提供测试程序。[完整依赖与部署说明](airsim/docs/setup.md)。

**验证范围：**原核心快照的 7 个 ROS 包已在本机独立构建通过；扩展版的新增 SDK、Agent 和打包后的全部组件尚未完成整套构建与新机器飞行验证。不能将“源码已补齐”视为“任意机器零配置即用”。[验证记录](airsim/docs/verification.md)。

## 本机成功基线

2026-11-11 配套 UE4 场景中，用户观察到全程没有碰壁或碰到障碍，最终到达目标；完整日志最后执行轨迹 68，FSM 随后进入 WAIT_TARGET。起步有交接重试与一次停稳恢复，之后完成任务。

| 参数 | 基线值 |
| --- | --- |
| ROS 2 / PX4 | Humble / v1.15.2 |
| 目标 ENU | `(0.15, 15.0, 2.0)` m |
| 起飞高度 | 2 m |
| 规划速度 / 加速度上限 | 0.5 m/s / 1.0 m/s² |
| 障碍膨胀 / 地图分辨率 | **0.5 m** / 0.1 m |
| 深度相机 | 200×150，FOV 90°，AirSim 安装 Z=1 m |
| 桥 | 单调控制定时器，`px4_timestamp_mode=receiver`，累计恢复上限 3 次 |

## 运行顺序

先按新机器实际网络地址调整 [settings.json](airsim/hw-ros2/config/airsim/settings.json) 中的 `LocalHostIp`，并将该配置复制到 Windows `%USERPROFILE%\Documents\AirSim\settings.json`。车辆名为 PX4，相机名为 CameraDepth。

1. 在 UE4 启动兼容的 AirSim 场景。
2. 终端一，在仓库根目录运行 `bash airsim/tools/run_agent.sh 8888`。
3. 终端二，进入 `airsim/px4v1.15.2/`，运行 `make px4_sitl_default none_iris`。
4. 其余 ROS 终端均 source `/opt/ros/humble/setup.bash` 和 `airsim/hw-ros2/ros2/install/setup.bash`。
5. 启动 `ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=<Windows主机IP>`。
6. 启动 `ros2 launch ego_planner rviz.launch.py`。
7. 启动 `ros2 launch ego_planner single_run_in_sim.launch.py`。
8. 启动桥，等待稳定悬停：

```bash
ros2 run hw_insight position_cmd_to_px4 \
  --ros-args -p mode:=ego -p fixed_height:=2.0 \
  -p px4_timestamp_mode:=receiver
```

9. 手动触发任务：

```bash
ros2 topic pub --once /traj_start_trigger geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}'
```

只运行一套 planner/server/bridge；固定高度参数用于起飞悬停，后续按 EGO 的三维轨迹执行。

## 核心源码入口

- [FSM 与恢复流程](airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp)
- [轨迹服务器](airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/traj_server.cpp)
- [地图与深度投影](airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp)
- [B 样条优化器](airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp)
- [PX4 控制桥](airsim/hw-ros2/ros2/src/hw_insight/hw_insight/position_cmd_to_px4.py)
- [仿真入口与目标配置](airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py)
- [规划与地图参数](airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.launch.py)
- [PX4 本地距离传感器修改](airsim/px4v1.15.2/src/modules/mavlink/streams/DISTANCE_SENSOR.hpp)

## 归档校验与许可证

```bash
python3 airsim/hw-ros2/tools/verify_snapshot.py
```

校验复制源码、配置和许可证的 SHA-256。协议消息中名称包含 Test 的 `.msg` 属于 PX4 消息定义，保留以维持消息包一致性；它们不是测试程序。

[第三方许可](airsim/hw-ros2/LICENSES/README.md)保留 EGO-Planner GPL v3、AirSim/rpclib、Eigen、PX4 和 DDS Agent 各自声明。规划器上游为 [ZJU FAST Lab ros2_version 分支](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)。原标签 `baseline-2026-11-11` 保留最初基线，main 是扩展后的工程归档。
