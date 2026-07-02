#!/usr/bin/env python3
"""
robot_gui — bảng điều khiển Start/Stop cho các chức năng của robot,
thay cho việc mở nhiều terminal rồi `ros2 launch` từng cái thủ công.

Chạy: ros2 run gui robot_gui
(cần source workspace TRƯỚC khi chạy — GUI kế thừa môi trường hiện tại
để gọi `ros2 launch main_bot ...`, không tự source gì cả)
"""
import os
import signal
import subprocess
import tkinter as tk
from tkinter import messagebox, ttk

# (tên hiển thị, lệnh, persistent)
# persistent=True  -> lệnh đứng chờ mãi (launch), cần bấm Stop để dừng
# persistent=False -> lệnh chạy 1 lần rồi tự thoát (vd docking)
ACTIONS = [
    ("Gazebo + Robot", ["ros2", "launch", "main_bot", "gazebo.launch.py"], True),
    ("SLAM Mapping", ["ros2", "launch", "main_bot", "slam_mapping.launch.py"], True),
    ("Nav2 + AMCL (bản đồ đã lưu)", ["ros2", "launch", "main_bot", "nav2_bringup.launch.py"], True),
    ("Nav2 + UWB (định vị anchor)", ["ros2", "launch", "main_bot", "nav2_uwb.launch.py"], True),
    ("RViz", ["ros2", "launch", "main_bot", "display.launch.py"], True),
    ("Docking (về trạm sạc)", ["ros2", "run", "main_bot", "dock_to_charge.py"], False),
]

# Các pattern tiến trình hay bị sót lại sau khi Ctrl+C không sạch —
# gz sim, rviz2, và cả chuỗi node Nav2/SLAM (đã gặp nhiều lần bị mồ côi
# gây robot tự di chuyển / TF lệch trong lúc phát triển project này).
CLEANUP_PATTERNS = [
    "gz sim",
    "rviz2",
    "ros2 launch main_bot",
    "ros2 run main_bot",
    "amcl",
    "map_server",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "smoother_server",
    "behavior_server",
    "waypoint_follower",
    "velocity_smoother",
    "collision_monitor",
    "lifecycle_manager",
    "ackermann_transform_node",
    "ekf_node",
    "uwb_sim_node",
    "uwb_localization_node",
    "robot_state_publisher",
    "parameter_bridge",
    "slam_toolbox",
]


class RobotGui(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("Robot Control Panel")
        self.geometry("560x460")
        self.processes = {}  # tên -> subprocess.Popen
        self._build_ui()
        self._poll()

    def _build_ui(self):
        tk.Label(self, text="Robot Control Panel", font=("Sans", 14, "bold")).pack(pady=(12, 6))

        rows_frame = tk.Frame(self)
        rows_frame.pack(fill="x", padx=12)

        self.status_labels = {}
        for name, cmd, persistent in ACTIONS:
            row = tk.Frame(rows_frame)
            row.pack(fill="x", pady=4)

            status = tk.Label(row, text="●", fg="gray", width=2)
            status.pack(side="left")
            self.status_labels[name] = status

            tk.Label(row, text=name, width=24, anchor="w").pack(side="left")

            tk.Button(
                row, text="Start", width=8,
                command=lambda n=name, c=cmd, p=persistent: self.start(n, c, p),
            ).pack(side="left", padx=4)

            tk.Button(
                row, text="Stop", width=8,
                command=lambda n=name: self.stop(n),
            ).pack(side="left")

        ttk.Separator(self, orient="horizontal").pack(fill="x", pady=12, padx=12)

        tk.Button(
            self,
            text="Dọn dẹp tiến trình cũ (Gazebo / RViz / Nav2 / SLAM...)",
            bg="#c0392b", fg="white",
            command=self.cleanup_stale,
        ).pack(pady=6, padx=12, fill="x")

        self.log = tk.Text(self, height=12, state="disabled", bg="#111111", fg="#33ff33")
        self.log.pack(fill="both", expand=True, padx=12, pady=(6, 12))

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _write_log(self, msg):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def start(self, name, cmd, persistent):
        proc = self.processes.get(name)
        if proc is not None and proc.poll() is None:
            self._write_log(f"[{name}] đang chạy rồi.")
            return
        try:
            # preexec_fn=os.setsid: tạo process group riêng, để Stop sau này
            # kill được HẾT cây con (ros2 launch đẻ ra rất nhiều tiến trình con).
            proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except FileNotFoundError as e:
            messagebox.showerror(
                "Lỗi",
                f"Không chạy được lệnh: {e}\n\nBạn đã source workspace trước khi mở GUI này chưa?\n"
                "(source /opt/ros/jazzy/setup.bash && source install/setup.bash)",
            )
            return
        self.processes[name] = proc
        self._write_log(f"[{name}] started (pid {proc.pid})")
        self.status_labels[name].configure(fg="green" if persistent else "orange")

    def stop(self, name):
        proc = self.processes.get(name)
        if proc is None or proc.poll() is not None:
            self._write_log(f"[{name}] không chạy.")
            self.status_labels[name].configure(fg="gray")
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._write_log(f"[{name}] không thoát sau 5s, buộc kill.")
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self._write_log(f"[{name}] stopped.")
        self.status_labels[name].configure(fg="gray")

    def cleanup_stale(self):
        if not messagebox.askyesno(
            "Xác nhận",
            "Sẽ kill TẤT CẢ tiến trình Gazebo/RViz/Nav2/SLAM đang chạy trên máy "
            "(kể cả không phải do GUI này mở). Tiếp tục?",
        ):
            return
        found_any = False
        for pattern in CLEANUP_PATTERNS:
            result = subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)
            if result.returncode == 0:
                found_any = True
        self.processes.clear()
        for status in self.status_labels.values():
            status.configure(fg="gray")
        self._write_log("Đã dọn dẹp tiến trình cũ." if found_any else "Không có tiến trình nào cần dọn.")

    def _poll(self):
        for name, _cmd, persistent in ACTIONS:
            proc = self.processes.get(name)
            if proc is not None and proc.poll() is not None:
                # tiến trình đã tự thoát (vd docking chạy xong, hoặc launch bị crash)
                self.status_labels[name].configure(fg="blue" if not persistent else "gray")
        self.after(1000, self._poll)

    def on_close(self):
        running = [n for n, p in self.processes.items() if p.poll() is None]
        if running and not messagebox.askyesno(
            "Thoát",
            f"Còn {len(running)} tiến trình đang chạy ({', '.join(running)}).\n"
            "Đóng GUI sẽ KHÔNG tự kill chúng — dùng nút Dọn dẹp trước nếu muốn tắt hết.\n"
            "Vẫn thoát?",
        ):
            return
        self.destroy()


def main():
    app = RobotGui()
    app.mainloop()


if __name__ == "__main__":
    main()
