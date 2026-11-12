#!/usr/bin/env bash
set -euo pipefail
archive_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
component="${1:-ros}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
case "$component" in
  ros)
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$archive_dir/hw-ros2/ros2"
    colcon build --packages-up-to ego_planner px4_msgs airsim_ros_pkgs hw_insight \
      --executor sequential --symlink-install --cmake-args -DBUILD_TESTING=OFF
    ;;
  agent)
    cmake -S "$archive_dir/Micro-XRCE-DDS-Agent-v2.4.2" \
      -B "$archive_dir/Micro-XRCE-DDS-Agent-v2.4.2/build" \
      -DUAGENT_BUILD_TESTS=OFF -DUAGENT_BUILD_CI_TESTS=OFF \
      -DUAGENT_BUILD_USAGE_EXAMPLES=OFF -DUAGENT_INSTALLER=OFF \
      -DCMAKE_INSTALL_PREFIX="$archive_dir/.local"
    cmake --build "$archive_dir/Micro-XRCE-DDS-Agent-v2.4.2/build" --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
    cmake --install "$archive_dir/Micro-XRCE-DDS-Agent-v2.4.2/build"
    ;;
  px4)
    if [[ ! -e "$archive_dir/px4v1.15.2/.git" ]]; then
      echo 'Run python3 airsim/tools/fetch_px4.py first.' >&2
      exit 1
    fi
    cd "$archive_dir/px4v1.15.2"
    make -j"$CMAKE_BUILD_PARALLEL_LEVEL" px4_sitl_default
    ;;
  *)
    echo 'Usage: bash airsim/tools/build.sh {ros|agent|px4}' >&2
    exit 2
    ;;
esac
