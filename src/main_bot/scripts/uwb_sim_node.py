#!/usr/bin/env python3
"""
uwb_sim_node — giả lập phần cứng UWB trong Gazebo.

Lấy ground-truth pose robot từ /ground_truth/odom (plugin OdometryPublisher
trong uwb_tag.xacro, bridge qua gz_bridge.yaml), tính khoảng cách 3D thật
tới từng anchor rồi cộng nhiễu Gaussian + lỗi NLOS/dropout, publish
sensor_msgs/Range trên /uwb/range_<i> — đúng dạng dữ liệu 1 driver UWB
thật sẽ đưa ra.

Lưu ý frame: OdometryPublisher (gz Harmonic, đã kiểm chứng) xuất pose
trong frame WORLD → gt_pose_frame="world": anchor giữ nguyên tọa độ world.
Nếu plugin version khác xuất pose tương đối so với chỗ spawn thì dùng
"relative" — anchor sẽ được quy sang frame map (gốc map = điểm spawn);
khoảng cách không đổi theo frame nên chỉ cần tag và anchor cùng frame.

Khi có phần cứng thật: chỉ cần thay node này bằng driver đọc UART/SPI
từ module DWM, giữ nguyên format topic → toàn bộ phần định vị phía sau
(uwb_localization_node + EKF) dùng lại nguyên vẹn.
"""
import math
import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range


class UwbSimNode(Node):

    def __init__(self):
        super().__init__('uwb_sim_node')

        self.declare_parameter('anchors_world', [0.0])
        self.declare_parameter('world_to_map_xy_yaw', [0.0, 0.0, 0.0])
        self.declare_parameter('gt_pose_frame', 'relative')  # relative | world
        self.declare_parameter('tag_height', 0.114)
        self.declare_parameter('range_rate_hz', 20.0)
        self.declare_parameter('range_noise_std', 0.05)
        self.declare_parameter('dropout_prob', 0.02)
        self.declare_parameter('nlos_prob', 0.03)
        self.declare_parameter('nlos_bias_min', 0.10)
        self.declare_parameter('nlos_bias_max', 0.40)
        self.declare_parameter('gt_topic', '/ground_truth/odom')

        flat = list(self.get_parameter('anchors_world').value)
        if len(flat) < 9 or len(flat) % 3 != 0:
            raise RuntimeError('anchors_world phải là [x,y,z]*N, N>=3 — kiểm tra uwb.yaml')
        anchors = [(flat[i], flat[i + 1], flat[i + 2]) for i in range(0, len(flat), 3)]

        # gt_pose_frame=relative: pose ground-truth tính từ chỗ spawn (= gốc
        # map) → quy anchor world→map để cùng frame với tag khi tính range
        if self.get_parameter('gt_pose_frame').value == 'relative':
            wx, wy, wyaw = self.get_parameter('world_to_map_xy_yaw').value
            c, s = math.cos(-wyaw), math.sin(-wyaw)
            anchors = [(c * (ax - wx) - s * (ay - wy),
                        s * (ax - wx) + c * (ay - wy), az)
                       for ax, ay, az in anchors]
        self.anchors = anchors

        self.tag_height = self.get_parameter('tag_height').value
        self.noise_std = self.get_parameter('range_noise_std').value
        self.dropout_prob = self.get_parameter('dropout_prob').value
        self.nlos_prob = self.get_parameter('nlos_prob').value
        self.nlos_bias = (self.get_parameter('nlos_bias_min').value,
                          self.get_parameter('nlos_bias_max').value)

        self.gt_xy = None  # vị trí (x, y) mới nhất của robot

        self.create_subscription(Odometry, self.get_parameter('gt_topic').value,
                                 self.gt_callback, 10)

        self.range_pubs = [
            self.create_publisher(Range, f'/uwb/range_{i}', 10)
            for i in range(len(self.anchors))
        ]

        rate = self.get_parameter('range_rate_hz').value
        self.create_timer(1.0 / rate, self.publish_ranges)
        self.get_logger().info(
            f'UWB sim: {len(self.anchors)} anchors, {rate:.0f}Hz, '
            f'σ={self.noise_std * 100:.0f}cm, NLOS={self.nlos_prob * 100:.0f}%')

    def gt_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.gt_xy = (p.x, p.y)

    def publish_ranges(self):
        if self.gt_xy is None:
            return
        tx, ty = self.gt_xy
        stamp = self.get_clock().now().to_msg()
        for i, (ax, ay, az) in enumerate(self.anchors):
            if random.random() < self.dropout_prob:
                continue
            d = math.sqrt((ax - tx) ** 2 + (ay - ty) ** 2 + (az - self.tag_height) ** 2)
            r = d + random.gauss(0.0, self.noise_std)
            if random.random() < self.nlos_prob:
                r += random.uniform(*self.nlos_bias)  # NLOS luôn làm range DÀI ra
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = f'uwb_anchor_{i}'
            msg.radiation_type = Range.ULTRASOUND  # không có hằng UWB, dùng tạm
            msg.field_of_view = 6.28
            msg.min_range = 0.0
            msg.max_range = 50.0
            msg.range = max(0.0, r)
            self.range_pubs[i].publish(msg)


def main():
    rclpy.init()
    node = UwbSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
