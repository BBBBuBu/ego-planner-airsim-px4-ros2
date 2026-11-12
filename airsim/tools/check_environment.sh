#!/usr/bin/env bash
set -u
archive_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
failed=0
for command in git cmake make gcc g++ python3; do
  if command -v "$command" >/dev/null 2>&1; then printf 'OK: %s\n' "$command"; else printf 'MISSING: %s\n' "$command"; failed=1; fi
done
for relative in hw-ros2/AirLib/include hw-ros2/MavLinkCom/include hw-ros2/external/rpclib/rpclib-2.3.0/include; do
  if [[ -d "$archive_dir/$relative" ]]; then printf 'OK: %s\n' "$relative"; else printf 'MISSING: %s\n' "$relative"; failed=1; fi
done
if [[ -f /opt/ros/humble/setup.bash ]]; then echo 'OK: ROS 2 Humble'; else echo 'MISSING: ROS 2 Humble'; failed=1; fi
if [[ -e "$archive_dir/px4v1.15.2/.git" ]]; then echo 'OK: PX4 source checkout'; else echo 'PENDING: run python3 airsim/tools/fetch_px4.py'; fi
printf '%s\n' 'UE4 scene/plugin and host IP must be configured on the target Windows machine.'
exit "$failed"
