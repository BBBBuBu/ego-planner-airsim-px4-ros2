import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        # world → world_ned: ENU → NED 静态旋转 (180° around Z)
        # world (ENU) 是 ego-planner 根坐标系
        # world_ned 是 PX4/AirSim NED 坐标系
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_ned_pub',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0.7071067811865476',
                '--qy', '0.7071067811865476',
                '--qz', '0', '--qw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'world_ned'
            ]
        ),

        # world → map: 静态，ego-planner 地图坐标系
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        ),

    ])
    return ld