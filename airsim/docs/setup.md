# 新机器部署与外部前提

## 系统依赖

使用 Ubuntu 22.04、ROS 2 Humble。先按 ROS 官方安装方式准备 Humble 桌面版与开发环境。以下为本项目的主要额外构建依赖，需在用户自己的机器明确执行安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-pip python3-colcon-common-extensions \
  python3-rosdep python3-empy libeigen3-dev libopencv-dev libpcl-dev \
  libyaml-cpp-dev libasio-dev libtinyxml2-dev libssl-dev \
  ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-message-filters \
  ros-humble-geographic-msgs ros-humble-mavros-msgs ros-humble-tf2-sensor-msgs \
  ros-humble-image-transport
```

安装 ROS 本身不由本仓库脚本自动执行，也不自动修改用户的 `.bashrc`、系统时间或系统包。

## PX4

在仓库根目录执行 `python3 airsim/tools/fetch_px4.py`，再按需在准备好的源码树中执行：

```bash
bash airsim/px4v1.15.2/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
```

此命令会安装系统/Python 依赖；应先阅读该脚本。使用 `--no-nuttx` 可以避开本地 setup 修改中依赖离线 ARM 工具链压缩包的分支；本项目运行的是 SITL。`--no-sim-tools` 不安装 Gazebo/jMAVSim，因为这里使用 UE4/AirSim。

随后运行 `bash airsim/tools/build.sh px4`。准备脚本锁定 Git 提交与版本标签并初始化子模块，保留本机 DISTANCE_SENSOR.hpp 和 setup/ubuntu.sh 的修改；已有不同版本或额外本地修改时停止，不做强制重置。

为保持核心仓库可控，完整 PX4 上游源码和其测试/仿真模型属于下载依赖，下载后仍被本仓库忽略规则排除。

## DDS Agent

`bash airsim/tools/build.sh agent` 使用归档的 2.4.2 核心源码，保留本地将 FastDDS 下载分支固定为 v2.12.2 的修改。CMake 可能下载 Fast-CDR、Fast-DDS、foonathan_memory、spdlog 和 Micro XRCE-DDS Client 等依赖，因此需要可用网络。

安装目标是仓库内 `airsim/.local/`，不需要 sudo install。`run_agent.sh` 仅为该进程设置本地库路径，避免混入其他 ROS 终端。端口 8888 需与 PX4 的 DDS 配置一致。

## AirSim SDK 与 ROS

本次已补齐 `hw-ros2/AirLib`、`MavLinkCom`、`external/rpclib/rpclib-2.3.0` 和三个核心库的 CMake 规则。`airsim_ros_pkgs/CMakeLists.txt` 中 ../../../ 指向的 SDK 根目录与原工程层级一致。

MavLinkCom 原构建规则无条件引入 MavLinkTest，本归档移除了这一项，从而不分发或构建测试程序。运行用 C++ 源码未因此改变。ROS 构建明确关闭 BUILD_TESTING；不要使用需要测试源码的构建选项。

`hw_insight` 被打包为仅包含位置桥的最小 Python ROS 包；未带上原教学工程的键盘控制、课堂 launch 或诊断测试脚本。桥主体与成功基线内容一致。

## UE4 与地图

UE4 引擎、Windows AirSim 插件、Asian_Village_Demo 等场景/地图资产需要单独取得并配置。它们不在本 Linux 核心源码归档中。本次没有在一台全新机器上验证 UE4 插件与归档 SDK 的兼容性，因此不能承诺任意 AirSim/UE4 版本均可直接运行。

将 `airsim/hw-ros2/config/airsim/settings.json` 放到 Windows 的 `%USERPROFILE%\Documents\AirSim\settings.json`，按实际主机修改 IP。不要把相机安装 Z=1 与飞机起飞高度或规划坐标混淆；grid_map.cpp 的相机外参必须与它配套。

## 时间

本机 WSL 使用 Windows/Hyper-V 同步，曾关闭内部 systemd-timesyncd 后恢复稳定。新机器应先核实宿主和 WSL 时间是否稳定；不要在其他原生 Linux 环境直接照搬关闭校时服务的操作。
