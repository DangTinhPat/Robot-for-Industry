#!/usr/bin/env python3

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Vị trí + hướng bến sạc trong FRAME MAP (không phải world Gazebo!).
# Bản đồ my_map được SLAM tạo bắt đầu từ chính điểm spawn/bến sạc
# → bến sạc = gốc map (0, 0, yaw 0). Nếu lưu bản đồ mới từ vị trí
# xuất phát khác thì phải cập nhật lại các giá trị này.
DOCK_X = 0.0
DOCK_Y = 0.0
DOCK_YAW = 0.0


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    rclpy.init()
    navigator = BasicNavigator()
    # localizer='bt_navigator': mặc định hàm này chờ node 'amcl' — hệ này
    # định vị bằng UWB (không có AMCL) nên sẽ treo vĩnh viễn. bt_navigator
    # có mặt ở cả 2 chế độ (nav2_bringup lẫn nav2_uwb) nên chờ nó là đủ.
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    dock_pose = PoseStamped()
    dock_pose.header.frame_id = 'map'
    dock_pose.header.stamp = navigator.get_clock().now().to_msg()
    dock_pose.pose.position.x = DOCK_X
    dock_pose.pose.position.y = DOCK_Y
    _, _, qz, qw = yaw_to_quaternion(DOCK_YAW)
    dock_pose.pose.orientation.z = qz
    dock_pose.pose.orientation.w = qw

    navigator.info(f'Docking -> ({DOCK_X}, {DOCK_Y}), yaw={DOCK_YAW:.3f} rad')
    navigator.goToPose(dock_pose)

    while not navigator.isTaskComplete():
        pass

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.info('Docking succeeded — robot đã về trạm sạc.')
        exit_code = 0
    elif result == TaskResult.CANCELED:
        navigator.warn('Docking bị hủy.')
        exit_code = 1
    else:
        navigator.error('Docking thất bại — không tới được trạm sạc.')
        exit_code = 1

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
