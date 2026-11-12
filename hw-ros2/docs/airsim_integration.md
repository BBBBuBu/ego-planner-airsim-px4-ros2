# AirSim ROS 接入

AirSim ROS 包位于 hw-ros2/ros2/src/airsim_ros_pkgs 与 airsim_interfaces，与本机路径一致。源码保持原样。

airsim_ros_pkgs/CMakeLists.txt 通过 ../../../ 定位 hw-ros2/，需要该目录下的 AirLib、MavLinkCom、external/rpclib、cmake 等完整 SDK 源码。本仓库只归档 ROS 接入包，完整 SDK 单独准备。

```text
hw-ros2/
  AirLib/                 # 外部 AirSim SDK
  MavLinkCom/             # 外部 AirSim SDK
  external/rpclib/        # 外部 AirSim SDK
  cmake/                  # 外部 AirSim SDK
  ros2/src/airsim_ros_pkgs/
  ros2/src/airsim_interfaces/
```

准备好配套 SDK 和 geographic_msgs、mavros_msgs、tf2_sensor_msgs、image_transport、cv_bridge 等依赖后，在 hw-ros2/ros2/ 运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to airsim_ros_pkgs --symlink-install
source install/setup.bash
```

原环境未保留 AirSim SDK 的有效 Git 元数据，无法提供精确 AirLib 上游提交；完整 AirSim SDK 的干净重建未做验证。

hw-ros2/config/airsim/settings.json 对应 Windows 的 %USERPROFILE%\Documents\AirSim\settings.json。它保留了本机测试 IP=172.27.208.1，换机器时应与 AirSim ROS host、PX4 连接设置共同调整。车辆名为 PX4，相机名为 CameraDepth。
