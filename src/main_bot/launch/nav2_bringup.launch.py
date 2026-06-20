"""
nav2_bringup.launch.py

Chạy sau gazebo.launch.py (đợi controllers ổn định ~10s).
Dùng map đã lưu → map_server + amcl (localization).

Thứ tự:
  t=0s   map_server + amcl + lifecycle_manager_localization
  t=0s   ackermann_transform_node
  t=8s   Nav2 stack (đợi amcl sẵn sàng)

Không dùng navigation_launch.py từ nav2_bringup vì nó hard-code
route_server + docking_server vào lifecycle_nodes (không thể tắt).
Thay vào đó launch từng node thủ công, chỉ gồm những gì cần thiết.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    pkg_main_bot = get_package_share_directory('main_bot')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='true = Gazebo, false = robot thực')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_file    = os.path.join(pkg_main_bot, 'maps', 'my_map.yaml')
    nav2_params = os.path.join(pkg_main_bot, 'config', 'nav2_param.yaml')
    transform_cfg = os.path.join(pkg_main_bot, 'config', 'ackermann_transform.yaml')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Localization: map_server + amcl ──────────────────────────────────────

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
        }],
    )

    # ── Ackermann transform ───────────────────────────────────────────────────
    ackermann_transform = Node(
        package='main_bot',
        executable='ackermann_transform_node',
        name='ackermann_transform',
        parameters=[transform_cfg, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── Nav2 navigation nodes (không có route_server / docking_server) ────────
    nav2_nodes = GroupAction(actions=[
        SetParameter('use_sim_time', use_sim_time),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[nav2_params],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart':    True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'velocity_smoother',
                    'collision_monitor',
                    'bt_navigator',
                    'waypoint_follower',
                ],
            }],
        ),
    ])

    return LaunchDescription([
        use_sim_time_arg,

        # t=0: localization + transform
        map_server,
        amcl,
        lifecycle_manager_loc,
        ackermann_transform,

        # t=8: Nav2 (đợi map_server + amcl active)
        TimerAction(period=8.0, actions=[nav2_nodes]),
    ])
