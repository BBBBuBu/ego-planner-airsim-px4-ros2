# ego-planner-airsim-px4-ros2

UE4 / AirSim / PX4 SITL 避障工程的原文件归档。

**按用户要求：仅复制、整理目录和筛选文件，不修改原工程，也不修改归档源码、CMake、launch、配置或包元数据的内容。**

```text
airsim/
├── hw-ros2/
│   ├── AirLib/
│   ├── MavLinkCom/
│   ├── external/rpclib/rpclib-2.3.0/
│   ├── cmake/
│   ├── ros2/src/
│   │   ├── ego-planner-swarm/
│   │   ├── hw_insight/
│   │   ├── airsim_ros_pkgs/
│   │   ├── airsim_interfaces/
│   │   └── px4_msgs/
│   ├── config/airsim/settings.json
│   ├── baseline/          # 来源、参数摘要和文件校验清单
│   └── LICENSES/
├── px4v1.15.2/            # 本地修改的源文件与许可证，不是完整固件
└── Micro-XRCE-DDS-Agent-v2.4.2/
```

例如，原工程 `/home/airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp` 对应本仓库 `airsim/hw-ros2/ros2/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp`。

## 归档范围

- EGO-Planner 地图、搜索、优化、FSM、轨迹管理和配套消息。
- PX4 桥、原包元数据、AirSim ROS 接入包和配置。
- AirLib、MavLinkCom、rpclib/Eigen 及 DDS Agent 的运行源码。
- PX4 本地 DISTANCE_SENSOR.hpp、Tools/setup/ubuntu.sh 和许可证，基准为 v1.15.2。
- README、许可证说明和校验清单是归档说明，不属于对用户运行文件的修改。

日志、缓存、build/install、录包、备份、测试程序和教学示例不收录。名称含 Test 的标准 PX4 `.msg` 是协议定义，仍保留以维持消息包一致性。

## 使用边界

这是一份**原样核心文件归档**，不是经过适配的一键安装工程。未修改原构建规则，因此被排除的测试/可选模块目录仍可能被原 CMake 或包配置引用；使用时需要配套原工程或完整上游依赖。UE4 引擎、插件和地图资产单独准备。

之前为打包新增的构建/依赖准备脚本及改写的构建元数据已经撤出本上传归档；CMake、setup.py、package.xml 已从原文件重新复制。

本机 2026-11-11 测试中，用户观察到无障碍碰撞并到达目标，日志最终进入 WAIT_TARGET。该记录属于已验证的原环境，不代表这份精简归档已在新机器独立构建或飞行验证。

校验清单位于 [source_manifest.json](airsim/hw-ros2/baseline/source_manifest.json)。上游许可证和组件声明见 [LICENSES](airsim/hw-ros2/LICENSES/README.md)。
