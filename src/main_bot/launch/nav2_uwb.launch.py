"""
nav2_uwb.launch.py — Nav2 định vị bằng UWB thay cho AMCL.

Chạy sau gazebo.launch.py (uwb_sim_node ở đó đã publish /uwb/range_*).

Kiến trúc định vị:
  /uwb/range_*  →  uwb_localization_node  →  /uwb/pose (frame map)
  /uwb/pose + wheel odom + IMU  →  ekf_global_node  →  TF map→odom
  (AMCL bị loại hoàn toàn — không còn phụ thuộc motion model differential
   vốn không khớp động học Ackermann)

map_server vẫn chạy để cấp bản đồ tĩnh cho global costmap.

Thứ tự (so le để tránh nghẽn CPU/DDS lúc khởi động — ekf_node từng bị kẹt
"Waiting for clock" khi start cùng lúc với cả cụm):
  t=0s   map_server + lifecycle  |  uwb_localization + ackermann_transform
  t=3s   ekf_global (cần /uwb/pose đã chảy + hệ đã bớt bận)
  t=12s  Nav2 stack (đợi TF map→odom ổn định — global_costmap sẽ không
         activate được nếu TF map chưa có)
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

    map_file      = os.path.join(pkg_main_bot, 'maps', 'my_map.yaml')
    nav2_params   = os.path.join(pkg_main_bot, 'config', 'nav2_param.yaml')
    transform_cfg = os.path.join(pkg_main_bot, 'config', 'ackermann_transform.yaml')
    uwb_cfg       = os.path.join(pkg_main_bot, 'config', 'uwb.yaml')
    ekf_global    = os.path.join(pkg_main_bot, 'config', 'ekf_global.yaml')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Bản đồ tĩnh (cho global costmap) ─────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {'yaml_filename': map_file, 'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    True,
            'node_names':   ['map_server'],
        }],
    )

    # ── Định vị UWB: multilateration + EKF toàn cục (map→odom) ──────────────
    uwb_localization = Node(
        package='main_bot',
        executable='uwb_localization_node.py',
        name='uwb_localization_node',
        parameters=[uwb_cfg, {'use_sim_time': use_sim_time}],
        output='screen',
        respawn=True, respawn_delay=5.0,
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        parameters=[ekf_global, {'use_sim_time': use_sim_time}],
        # odometry/filtered → odometry/global: /odom đã là của EKF cục bộ.
        # set_pose → /initialpose: nhận reset từ uwb_localization_node (tự
        # hồi phục khi bị bế đi chỗ khác) VÀ từ nút 2D Pose Estimate của
        # RViz — giống thao tác với AMCL trước đây.
        remappings=[('odometry/filtered', 'odometry/global'),
                    ('set_pose', '/initialpose')],
        output='screen',
        # node chết (máy quá tải, OOM...) → tự dậy lại; nếu thiếu node này
        # thì không có TF map→odom và global costmap sẽ không activate
        respawn=True, respawn_delay=5.0,
    )

    # ── Ackermann transform ───────────────────────────────────────────────────
    ackermann_transform = Node(
        package='main_bot',
        executable='ackermann_transform_node',
        name='ackermann_transform',
        parameters=[transform_cfg, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── Nav2 navigation nodes (giống nav2_bringup.launch.py, không AMCL) ─────
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

        # t=0: bản đồ + multilateration + transform
        map_server,
        lifecycle_manager_loc,
        uwb_localization,
        ackermann_transform,

        # t=3: EKF toàn cục (so le tránh nghẽn khởi động)
        TimerAction(period=3.0,  actions=[ekf_global_node]),

        # t=12: Nav2 (đợi map_server active + TF map→odom ổn định)
        TimerAction(period=12.0, actions=[nav2_nodes]),
    ])
