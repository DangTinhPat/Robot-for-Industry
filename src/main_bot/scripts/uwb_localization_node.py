#!/usr/bin/env python3
"""
uwb_localization_node — định vị tuyệt đối từ range UWB (multilateration).

Đầu vào : /uwb/range_<i>  (sensor_msgs/Range, từ uwb_sim_node hoặc driver thật)
          /imu            (heading — UWB 1-tag không tự đo được hướng)
Đầu ra  : /uwb/pose       (PoseWithCovarianceStamped, frame `map`)
          → ekf_global_node fuse với odometry để phát TF map→odom (thay AMCL).

Thuật toán mỗi chu kỳ (20Hz):
  1. Gom các range còn tươi (< range_timeout), chiếu 3D → 2D bằng độ cao
     anchor/tag đã biết: h = sqrt(r² − Δz²), trọng số theo hệ số khuếch đại
     nhiễu khi chiếu (σ_h = σ_r · r/h — anchor ngay trên đầu bị phạt nặng).
  2. Nghiệm thô closed-form (trừ phương trình từng cặp → hệ tuyến tính),
     dùng khi chưa có nghiệm trước đó.
  3. Tinh chỉnh Gauss-Newton có trọng số.
  4. Loại outlier theo consensus tổ hợp: nếu nghiệm toàn bộ anchor có
     residual > outlier_sigma thì duyệt các tổ hợp bỏ-1 rồi bỏ-2 anchor,
     lấy tổ hợp nhiều anchor nhất vượt ngưỡng với RMS nhỏ nhất (bắt được
     cả trường hợp 2 anchor NLOS cùng lúc mà cách bỏ-dần-từng-cái hay
     nhận nhầm anchor tốt).
  5. Covariance = s²·(JᵀWJ)⁻¹ (GDOP thật) — nghiệm hình học xấu tự khai
     "tôi không chắc" để EKF hạ trọng số.
  6. Gate nhảy vị trí: nghiệm dịch nhanh hơn max_speed → thổi phồng
     covariance ×25 thay vì vứt bỏ (EKF tự cân).
"""
import math
from itertools import combinations

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, Range


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UwbLocalizationNode(Node):

    def __init__(self):
        super().__init__('uwb_localization_node')

        self.declare_parameter('anchors_world', [0.0])
        self.declare_parameter('world_to_map_xy_yaw', [0.0, 0.0, 0.0])
        self.declare_parameter('tag_height', 0.114)
        self.declare_parameter('range_noise_std', 0.05)
        self.declare_parameter('solve_rate_hz', 20.0)
        self.declare_parameter('range_timeout', 0.30)
        self.declare_parameter('min_anchors', 3)
        self.declare_parameter('outlier_sigma', 3.0)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('min_pose_std', 0.05)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('imu_yaw_offset', 0.0)
        self.declare_parameter('yaw_std', 0.05)

        flat = list(self.get_parameter('anchors_world').value)
        if len(flat) < 9 or len(flat) % 3 != 0:
            raise RuntimeError('anchors_world phải là [x,y,z]*N, N>=3 — kiểm tra uwb.yaml')
        anchors_world = np.array(flat).reshape(-1, 3)

        # Quy đổi anchor world → map: p_map = R(−yaw)·(p_world − t)
        # (tương đương bước "khảo sát tọa độ anchor trong frame bản đồ" ngoài đời)
        wx, wy, wyaw = self.get_parameter('world_to_map_xy_yaw').value
        c, s = math.cos(-wyaw), math.sin(-wyaw)
        self.anchors = anchors_world.copy()
        dx = anchors_world[:, 0] - wx
        dy = anchors_world[:, 1] - wy
        self.anchors[:, 0] = c * dx - s * dy
        self.anchors[:, 1] = s * dx + c * dy
        self.n_anchors = len(self.anchors)

        self.tag_height = self.get_parameter('tag_height').value
        self.sigma_r = self.get_parameter('range_noise_std').value
        self.range_timeout = self.get_parameter('range_timeout').value
        self.min_anchors = int(self.get_parameter('min_anchors').value)
        self.outlier_sigma = self.get_parameter('outlier_sigma').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_pose_var = self.get_parameter('min_pose_std').value ** 2
        self.use_imu_yaw = self.get_parameter('use_imu_yaw').value
        self.imu_yaw_offset = self.get_parameter('imu_yaw_offset').value
        self.yaw_std = self.get_parameter('yaw_std').value

        # cache range mới nhất theo anchor: (range, t_nhận_theo_giây)
        self.latest = [None] * self.n_anchors
        self.imu_yaw = None
        self.imu_time = None
        self.last_solution = None   # (x, y)
        self.last_solve_time = None

        for i in range(self.n_anchors):
            self.create_subscription(
                Range, f'/uwb/range_{i}',
                lambda msg, idx=i: self.range_callback(msg, idx), 10)
        if self.use_imu_yaw:
            self.create_subscription(
                Imu, self.get_parameter('imu_topic').value, self.imu_callback, 50)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb/pose', 10)

        rate = self.get_parameter('solve_rate_hz').value
        self.create_timer(1.0 / rate, self.solve)

        self.get_logger().info(
            f'UWB localization: {self.n_anchors} anchors (frame map), '
            f'σ_r={self.sigma_r * 100:.0f}cm, outlier>{self.outlier_sigma}σ')

    # ── callbacks ───────────────────────────────────────────────────────────

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def range_callback(self, msg: Range, idx: int):
        self.latest[idx] = (msg.range, self.now_sec())

    def imu_callback(self, msg: Imu):
        self.imu_yaw = yaw_from_quaternion(msg.orientation) + self.imu_yaw_offset
        self.imu_time = self.now_sec()

    # ── solver ──────────────────────────────────────────────────────────────

    def gather(self):
        """Range tươi → (chỉ số anchor, khoảng cách 2D chiếu, σ chiếu)."""
        now = self.now_sec()
        idx, h, sigma = [], [], []
        for i, entry in enumerate(self.latest):
            if entry is None or now - entry[1] > self.range_timeout:
                continue
            r = entry[0]
            dz = self.anchors[i, 2] - self.tag_height
            if r <= abs(dz) + 0.01:
                continue  # range ngắn hơn chênh cao — vô nghĩa hình học
            h2d = math.sqrt(r * r - dz * dz)
            idx.append(i)
            h.append(h2d)
            sigma.append(self.sigma_r * max(r / h2d, 1.0))
        return np.array(idx, dtype=int), np.array(h), np.array(sigma)

    def linear_seed(self, A, h):
        """Nghiệm thô: trừ phương trình vòng tròn đầu tiên → hệ tuyến tính."""
        x0, y0 = A[0, 0], A[0, 1]
        M = 2.0 * (A[1:, :2] - A[0, :2])
        b = (A[1:, 0] ** 2 + A[1:, 1] ** 2 - x0 ** 2 - y0 ** 2
             - h[1:] ** 2 + h[0] ** 2)
        sol, *_ = np.linalg.lstsq(M, b, rcond=None)
        return sol

    def gauss_newton(self, p, A, h, sigma):
        """Tinh chỉnh WLS; trả (p, J, W, residual chuẩn hóa)."""
        W = np.diag(1.0 / sigma ** 2)
        J = None
        f = None
        for _ in range(10):
            d = A[:, :2] - p            # vector anchor→nghiệm (đảo dấu không sao)
            pred = np.linalg.norm(d, axis=1)
            pred = np.maximum(pred, 1e-6)
            f = pred - h
            J = -d / pred[:, None]      # ∂pred/∂p
            JtW = J.T @ W
            try:
                delta = np.linalg.solve(JtW @ J, -JtW @ f)
            except np.linalg.LinAlgError:
                return None
            p = p + delta
            if np.linalg.norm(delta) < 1e-4:
                break
        return p, J, W, f / sigma

    def solve(self):
        idx, h, sigma = self.gather()
        if len(idx) < self.min_anchors:
            self.get_logger().warn(
                f'Chỉ {len(idx)}/{self.n_anchors} anchor có range tươi — bỏ chu kỳ',
                throttle_duration_sec=5.0)
            return

        A_all = self.anchors[idx]
        seed = (np.array(self.last_solution) if self.last_solution is not None
                else self.linear_seed(A_all, h))

        # Consensus tổ hợp: thử toàn bộ anchor trước; nếu residual vượt
        # ngưỡng thì duyệt các tổ hợp bỏ-1, rồi bỏ-2, chọn tổ hợp NHIỀU
        # anchor nhất đạt ngưỡng với RMS residual nhỏ nhất. Bắt được cả
        # 2 anchor NLOS cùng lúc (~1.3%/chu kỳ với 6 anchor @3% NLOS) mà
        # cách bỏ-dần-từng-cái dễ nhận nhầm anchor tốt.
        n = len(h)
        best = None      # (rms, p, J, W, norm_res)
        for drop in range(0, min(3, n - self.min_anchors + 1)):
            for keep in combinations(range(n), n - drop):
                k = list(keep)
                result = self.gauss_newton(seed, A_all[k], h[k], sigma[k])
                if result is None:
                    continue
                p, J, W, norm_res = result
                if np.max(np.abs(norm_res)) > self.outlier_sigma:
                    continue
                rms = float(np.sqrt(np.mean(norm_res ** 2)))
                if best is None or rms < best[0]:
                    best = (rms, p, J, W, norm_res)
            if best is not None:
                break    # ưu tiên tổ hợp nhiều anchor nhất
        if best is None:
            # không tổ hợp nào sạch — dùng nghiệm toàn bộ, covariance sẽ
            # tự phồng theo residual (bước scale bên dưới)
            result = self.gauss_newton(seed, A_all, h, sigma)
            if result is None:
                return
            p, J, W, norm_res = result
        else:
            _, p, J, W, norm_res = best

        # Covariance từ hình học (GDOP), phồng theo residual thực tế
        try:
            cov2 = np.linalg.inv(J.T @ W @ J)
        except np.linalg.LinAlgError:
            return
        dof = max(len(norm_res) - 2, 1)
        scale = max(1.0, float(norm_res @ norm_res) / dof)
        cov2 = cov2 * scale
        # Sàn covariance: GDOP với 6 anchor cho ra ~2-3cm — quá lạc quan so
        # với thực tế NLOS, làm EKF bám theo nhiễu (robot "rung" khi đứng
        # yên). Không bao giờ khai chắc hơn min_pose_std.
        cov2[0, 0] = max(cov2[0, 0], self.min_pose_var)
        cov2[1, 1] = max(cov2[1, 1], self.min_pose_var)

        # Gate nhảy vị trí phi vật lý (multilateration glitch / NLOS lọt lưới)
        now = self.now_sec()
        if self.last_solution is not None and self.last_solve_time is not None:
            dt = max(now - self.last_solve_time, 1e-3)
            jump = math.dist(p, self.last_solution)
            if jump > self.max_speed * dt + 4.0 * self.sigma_r:
                cov2 = cov2 * 25.0
                self.get_logger().warn(
                    f'UWB nhảy {jump:.2f}m/{dt:.2f}s — phồng covariance',
                    throttle_duration_sec=2.0)
        self.last_solution = (float(p[0]), float(p[1]))
        self.last_solve_time = now

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(p[0])
        msg.pose.pose.position.y = float(p[1])

        cov = [0.0] * 36
        cov[0] = float(cov2[0, 0])
        cov[1] = float(cov2[0, 1])
        cov[6] = float(cov2[1, 0])
        cov[7] = float(cov2[1, 1])
        cov[14] = cov[21] = cov[28] = 1e6   # z / roll / pitch: không đo

        imu_fresh = (self.use_imu_yaw and self.imu_yaw is not None
                     and self.imu_time is not None and now - self.imu_time < 0.5)
        if imu_fresh:
            msg.pose.pose.orientation.z = math.sin(self.imu_yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(self.imu_yaw / 2.0)
            cov[35] = self.yaw_std ** 2
        else:
            msg.pose.pose.orientation.w = 1.0
            cov[35] = 1e6                    # yaw không tin được → EKF bỏ qua
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = UwbLocalizationNode()
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
