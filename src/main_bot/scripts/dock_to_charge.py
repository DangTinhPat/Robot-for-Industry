#!/usr/bin/env python3
"""
dock_to_charge.py — đưa robot quay về trạm sạc (docking).

Gửi 1 goal duy nhất qua Nav2 (navigate_to_pose) tới đúng vị trí bến sạc
(Room B, tường Nam, x=3.75 y=-4.175, hướng Bắc — khớp pose robot spawn
trong gazebo.launch.py). Chạy một lần rồi thoát — không phải service
thường trực (BasicNavigator tự spin nội bộ nên không an toàn khi chạy
trong callback lặp lại, xem bài học từ frontier_explorer trước đây).

Yêu cầu: Nav2 đã chạy sẵn (nav2_bringup.launch.py hoặc tương đương).

Dùng: ros2 run main_bot dock_to_charge.py
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Vị trí + hướng bến sạc — khớp robot spawn trong gazebo.launch.py
DOCK_X = 3.75
DOCK_Y = -4.175
DOCK_YAW = 1.5708  # hướng Bắc, khớp miệng bến sạc mở về phía Bắc


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

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
