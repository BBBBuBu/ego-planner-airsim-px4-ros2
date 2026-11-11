# AirSim ROS 接入覆盖层

本目录收录成功环境中的 airsim_ros_pkgs 与 airsim_interfaces，源码原样归档。

airsim_ros_pkgs/CMakeLists.txt 通过相对路径 ../../../ 定位 AirSim 根目录，需要其 AirLib、MavLinkCom、external/rpclib、cmake 等源码。它不是可脱离 AirSim 单独构建的 ROS wrapper。

在单独准备并构建好配套 AirSim 源码后，将本覆盖层的两个 ROS 包放入该 AirSim 工程的 ros2/src/ 下，然后在其 ros2/ 工作区构建。覆盖已有包前自行备份；推荐使用独立 AirSim 工作副本。

```text
<AirSim根目录>/
  AirLib/
  MavLinkCom/
  external/rpclib/
  cmake/
  ros2/src/airsim_ros_pkgs/
  ros2/src/airsim_interfaces/
```

```bash
source /opt/ros/humble/setup.bash
cd <AirSim根目录>/ros2
colcon build --packages-up-to airsim_ros_pkgs --symlink-install
source install/setup.bash
```

需要 geographic_msgs、mavros_msgs、tf2_sensor_msgs、image_transport、cv_bridge 等 ROS 依赖。本机 AirSim SDK 未保留可用 Git 元数据，因此本仓库无法提供精确 AirLib 上游提交；未在干净的完整 AirSim SDK 上重建验证本覆盖层。

config/airsim/settings.json 保留了本机验证时的 LocalHostIp=172.27.208.1。换机器时应与 AirSim ROS 的 host、PX4 网络连接配置共同调整。UE4 中车辆名必须为 PX4，相机名必须为 CameraDepth。
