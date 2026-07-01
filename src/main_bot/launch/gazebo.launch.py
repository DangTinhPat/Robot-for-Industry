import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_path = get_package_share_directory('main_bot')

    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    cart_xacro_file = os.path.join(pkg_path, 'description', 'cart.urdf.xacro')
    cart_description = xacro.process_file(cart_xacro_file).toxml()
    world_file    = os.path.join(pkg_path, 'worlds', 'warehouse.sdf')
    bridge_config = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    ekf_params    = os.path.join(pkg_path, 'config', 'ekf.yaml')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'True',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'dvt_robot', '-topic', 'robot_description',
                   '-x', '15', '-y', '-16.7', '-z', '0.1',
                   '-Y', '1.5708'],
        output='screen',
    )

    # Cargo carts — one parked at each receiving dock (x=15/26.25/37.5, y=14).
    # Passive props for now (free-rolling wheels, fixed hook) — no controller.
    # Facing south (-Y) so the front hook points into the room, toward the
    # aisle the robot approaches from.
    cart_dock_positions = [('cart_1', 15.0), ('cart_2', 26.25), ('cart_3', 37.5)]
    cart_spawners = [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', name, '-string', cart_description,
                       '-x', str(x), '-y', '14', '-z', '0.1',
                       '-Y', '-1.5708'],
            output='screen',
        )
        for name, x in cart_dock_positions
    ]

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    asc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen',
    )

    # EKF: fuse encoder odom + IMU → /odom + TF odom→base_footprint
    # Thay thế odom_tf_relay — EKF tự phát TF fused
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odom')],
        output='screen',
    )

    # Teleop: Twist → TwistStamped cho ackermann_steering_controller
    # Subscribe /teleop_cmd_vel (KHÔNG phải /cmd_vel) để tránh conflict với
    # ackermann_transform_node khi chạy Nav2 (cả hai đều publish lên /reference).
    # Dùng: ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    #         --ros-args -r cmd_vel:=/teleop_cmd_vel
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('cmd_vel_in',  '/teleop_cmd_vel'),
            ('cmd_vel_out', '/ackermann_steering_controller/reference'),
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        bridge_node,
        TimerAction(period=6.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=cart_spawners),
        TimerAction(period=10.0, actions=[jsb_spawner]),
        TimerAction(period=12.0, actions=[asc_spawner]),
        TimerAction(period=14.0, actions=[ekf_node]),
        TimerAction(period=14.0, actions=[twist_stamper]),
    ])
