#!/usr/bin/env python3
"""Record published wheel/fused odometry and simulator reference, then align/plot.

Start this BEFORE fusion_node. Add --live for an updating desktop plot.
Stop with Ctrl+C or close the live window to save the same four result files.
No simulator reference measurement is sent back to the filter.
"""
import argparse
import csv
import json
import math
import time
import textwrap
from pathlib import Path

import numpy as np


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def build_comparison(records):
    """One shared alignment/calculation for live plots and final files."""
    for source, rows in records.items():
        if len(rows) < 3:
            raise ValueError(f"Waiting for at least three {source} samples.")
        if any(b[0] < a[0] for a, b in zip(rows, rows[1:])):
            raise ValueError("Simulation time reset during recording. Record one uninterrupted segment.")
        if len({row[4] for row in rows}) != 1:
            raise ValueError("Odometry frame changed: record one trajectory segment at a time.")
    wheel = {row[0]: row for row in records["wheel"]}
    fused = {row[0]: row for row in records["fused"]}
    if records["wheel"][0][4] != records["fused"][0][4]:
        raise ValueError("Wheel and fused frame IDs differ.")
    reference = {row[0]: row for row in records["reference"]}
    ref_stamps = np.array(sorted(reference), dtype=np.int64)
    paired = sorted(set(wheel) & set(fused))
    paired = [t for t in paired if ref_stamps[0] <= t <= ref_stamps[-1]]
    if len(paired) < 3:
        raise ValueError("Too few timestamp-matched samples inside the reference time range.")
    first = wheel[paired[0]]
    if math.hypot(first[1], first[2]) > 1e-6 or abs(first[3]) > 1e-6:
        raise ValueError("Missed the initial local origin. Start recorder first, then restart fusion_node.")
    origin_ns = paired[0]
    rt = (ref_stamps - origin_ns).astype(float) * 1e-9
    t = (np.array(paired, dtype=np.int64) - origin_ns).astype(float) * 1e-9
    ref = np.array([[reference[int(stamp)][i] for i in (1, 2, 3)] for stamp in ref_stamps])
    ref[:, 2] = np.unwrap(ref[:, 2])
    # Interpolate the untouched simulator reference at the estimator timestamps.
    aligned = np.column_stack([np.interp(t, rt, ref[:, i]) for i in range(3)])
    x0, y0, yaw0 = aligned[0]
    dx, dy = aligned[:, 0] - x0, aligned[:, 1] - y0
    local_ref = np.column_stack([
        math.cos(yaw0) * dx + math.sin(yaw0) * dy,
        -math.sin(yaw0) * dx + math.cos(yaw0) * dy,
        wrap(aligned[:, 2] - yaw0),
    ])
    local_wheel = np.array([[wheel[stamp][i] for i in (1, 2, 3)] for stamp in paired])
    local_fused = np.array([[fused[stamp][i] for i in (1, 2, 3)] for stamp in paired])
    metrics = {"samples": len(paired), "duration_s": float(t[-1]),
               "reference_origin": {"x": float(x0), "y": float(y0), "yaw_rad": float(yaw0)}}
    errors = {}
    for name, estimate in (("wheel", local_wheel), ("fused", local_fused)):
        position = np.linalg.norm(estimate[:, :2] - local_ref[:, :2], axis=1)
        heading = wrap(estimate[:, 2] - local_ref[:, 2])
        errors[name] = (position, heading)
        metrics[name] = {
            "position_rmse_m": float(np.sqrt(np.mean(position ** 2))),
            "heading_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(heading ** 2)))),
            "final_position_error_m": float(position[-1]),
            "final_heading_error_deg": float(np.rad2deg(abs(heading[-1]))),
            "final_distance_from_start_m": float(np.linalg.norm(estimate[-1, :2])),
        }
    metrics["reference_final_distance_from_start_m"] = float(np.linalg.norm(local_ref[-1, :2]))
    return {"time": t, "reference": local_ref, "wheel": local_wheel,
            "fused": local_fused, "errors": errors, "metrics": metrics}


# Same identity/colour in every panel. The dashed wheel curve stays identifiable.
SERIES = {
    "reference": ("Simulator reference", "tab:blue", "-"),
    "wheel": ("Wheels", "tab:orange", "--"),
    "fused": ("EKF", "tab:green", "-"),
}


def create_plot(plt, live=False):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=not live)
    lines = {"trajectory": {}, "heading": {}, "position": {}, "error": {}}
    for source, (label, colour, style) in SERIES.items():
        options = {"label": label, "color": colour, "linestyle": style, "linewidth": 1.5}
        lines["trajectory"][source], = axes[0, 0].plot([], [], **options)
        lines["heading"][source], = axes[0, 1].plot([], [], **options)
        if source != "reference":
            lines["position"][source], = axes[1, 0].plot([], [], **options)
            lines["error"][source], = axes[1, 1].plot([], [], **options)
    axes[0, 0].set(xlabel="Local X (m)", ylabel="Local Y (m)", title="Trajectory")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 1].set(xlabel="Simulation time since alignment (s)", ylabel="Heading (degrees)", title="Heading")
    axes[1, 0].set(xlabel="Time (s)", ylabel="Position error (mm)", title="Position error")
    axes[1, 1].set(xlabel="Time (s)", ylabel="Signed heading error (degrees)", title="Heading error")
    axes[1, 1].axhline(0.0, color="0.5", linewidth=0.6, label="_zero")
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.legend()
    if live:
        fig.subplots_adjust(top=0.80, bottom=0.08, left=0.08, right=0.97, hspace=0.40, wspace=0.28)
        fig.suptitle("AURA live comparison", y=0.995)
        fig.canvas.manager.set_window_title("AURA live comparison")
    return fig, axes, lines


def draw_comparison(plot, data, max_points=None):
    fig, axes, lines = plot
    t = data["time"]
    indexes = np.arange(len(t))
    if max_points is not None and len(t) > max_points:
        # Limit GUI workload, preserving endpoints and important error extrema.
        important = [int(np.argmax(error[0])) for error in data["errors"].values()]
        important += [int(np.argmax(np.abs(error[1]))) for error in data["errors"].values()]
        indexes = np.unique(np.r_[np.linspace(0, len(t)-1, max_points, dtype=int), important])
    for source in SERIES:
        pose = data[source]
        lines["trajectory"][source].set_data(pose[indexes, 0], pose[indexes, 1])
        lines["heading"][source].set_data(t[indexes], np.rad2deg(np.unwrap(pose[:, 2]))[indexes])
        if source != "reference":
            position, heading = data["errors"][source]
            lines["position"][source].set_data(t[indexes], position[indexes] * 1000.0)
            lines["error"][source].set_data(t[indexes], np.rad2deg(heading[indexes]))
    for axis in axes.flat:
        axis.relim()
        axis.autoscale_view()
    # set_ylim disables automatic Y scaling. Recompute the upper bound on
    # every update so the first stationary samples cannot freeze this panel.
    position_max_mm = max(float(np.max(error[0])) for error in data["errors"].values()) * 1000.0
    axes[1, 0].set_ylim(0.0, max(1.0, 1.05 * position_max_mm))
    fig.canvas.draw_idle()


class LivePlot:
    """Display current sensor readings plus correctly time/frame-aligned poses."""
    def __init__(self, plt):
        self.plt = plt
        self.plot = create_plot(plt, live=True)
        self.fig = self.plot[0]
        self.sensor_text = self.fig.text(0.03, 0.955, "Waiting for wheel/gyro readings", fontsize=10)
        self.motion_text = self.fig.text(0.03, 0.925, "Waiting for wheel/EKF odometry", fontsize=10)
        self.status_text = self.fig.text(0.03, 0.895, "Start fusion_node after this recorder.", fontsize=10)

    def update(self, records, latest):
        newest = max((item[0] for item in latest.values()), default=0)
        def value(name):
            item = latest.get(name)
            if item is None or not math.isfinite(item[1]):
                return "--"
            suffix = " (old)" if newest-item[0] > 200_000_000 else ""
            return f"{item[1]:.3f}{suffix}"
        self.sensor_text.set_text(
            f"Latest samples: left {value('left')} | right {value('right')} rad/s | gyro {value('gyro')} rad/s")
        self.motion_text.set_text(
            f"Speed: wheels {value('wheel_v')} | EKF {value('fused_v')} m/s     "
            f"Turn: wheels {value('wheel_w')} | EKF {value('fused_w')} rad/s")
        try:
            data = build_comparison(records)
            draw_comparison(self.plot, data, max_points=2000)
            epw, ehw = data["errors"]["wheel"]
            epf, ehf = data["errors"]["fused"]
            self.status_text.set_text(
                f"Aligned t={data['time'][-1]:.2f}s | position: wheels {epw[-1]*1000:.2f}, EKF {epf[-1]*1000:.2f} mm | "
                f"heading: wheels {math.degrees(ehw[-1]):+.2f}, EKF {math.degrees(ehf[-1]):+.2f} deg")
        except ValueError as error:
            self.status_text.set_text(textwrap.fill(str(error), width=115))
        self.fig.canvas.draw_idle()


def analyse(records, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # Preserve observations even if the comparison cannot be aligned safely.
    with (output / "raw_odometry.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["source", "stamp_ns", "x", "y", "yaw_rad", "frames"])
        for source, rows in records.items():
            for row in rows:
                writer.writerow([source, *row])
    data = build_comparison(records)
    t, local_ref, local_wheel, local_fused = (
        data["time"], data["reference"], data["wheel"], data["fused"])
    errors, metrics = data["errors"], data["metrics"]
    with (output / "metrics.json").open("w") as handle:
        json.dump(metrics, handle, indent=2)
    combined = np.column_stack([
        t, local_ref, local_wheel, local_fused,
        errors["wheel"][0], errors["fused"][0],
        np.rad2deg(errors["wheel"][1]), np.rad2deg(errors["fused"][1]),
    ])
    np.savetxt(output / "aligned_comparison.csv", combined, delimiter=",", comments="",
               header="time_s,ref_x,ref_y,ref_yaw,wheel_x,wheel_y,wheel_yaw,"
                      "fused_x,fused_y,fused_yaw,wheel_position_error_m,fused_position_error_m,"
                      "wheel_heading_error_deg,fused_heading_error_deg")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plot = create_plot(plt)
    draw_comparison(plot, data)
    fig = plot[0]
    fig.savefig(output / "comparison.png", dpi=160)
    plt.close(fig)
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="aura_fusion_results")
    parser.add_argument("--live", action="store_true", help="Show updating plots and current wheel/gyro readings")
    parser.add_argument("--refresh-hz", type=float, default=5.0, help="GUI redraw rate, 0.5 to 10 Hz (default: 5)")
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.refresh_hz) or not 0.5 <= args.refresh_hz <= 10.0:
        parser.error("--refresh-hz must be between 0.5 and 10")
    live_plot = None
    if args.live:
        try:
            import matplotlib
            # pause() calls show() repeatedly; keep it from raising the window.
            matplotlib.rcParams["figure.raise_window"] = False
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            live_plot = LivePlot(plt)
            plt.show(block=False)
        except (ImportError, RuntimeError) as error:
            raise SystemExit("Live mode needs a desktop display and python3-tk. "
                             "Install with: sudo apt install python3-tk. Details: " + str(error)) from error
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState, Imu
    from rclpy.qos import qos_profile_sensor_data

    class Recorder(Node):
        def __init__(self):
            super().__init__("aura_fusion_comparison")
            self.records = {"reference": [], "wheel": [], "fused": []}
            self.latest = {}
            self.subscriptions_kept = []
            for label, topic in (("reference", "/odom"), ("wheel", "/aura/odom_wheel"),
                                 ("fused", "/aura/odom_fused")):
                self.subscriptions_kept.append(self.create_subscription(
                    Odometry, topic, lambda msg, source=label: self.record(source, msg), 100))
            if args.live:
                self.subscriptions_kept.append(self.create_subscription(
                    JointState, "/joint_states", self.wheels, qos_profile_sensor_data))
                self.subscriptions_kept.append(self.create_subscription(
                    Imu, "/imu", self.gyro, qos_profile_sensor_data))
            self.get_logger().info("Recording. Start fusion_node now; Ctrl+C or close the live window to save.")

        def wheels(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            for key, name in (("left", "joint_lw"), ("right", "joint_rw")):
                if name in msg.name:
                    index = msg.name.index(name)
                    if index < len(msg.velocity):
                        self.latest[key] = (stamp, float(msg.velocity[index]))

        def gyro(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            valid = msg.header.frame_id == "imu_link" and msg.angular_velocity_covariance[0] >= 0.0
            self.latest["gyro"] = (stamp, float(msg.angular_velocity.z) if valid else float("nan"))

        def record(self, source, msg):
            q = msg.pose.pose.orientation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if not math.isfinite(norm) or norm < 1e-6:
                return
            x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
            yaw = math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
            px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
            if not all(math.isfinite(v) for v in (px, py, yaw)):
                return
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            frames = msg.header.frame_id + "->" + msg.child_frame_id
            self.records[source].append((stamp, px, py, yaw, frames))
            if source != "reference":
                self.latest[source+"_v"] = (stamp, float(msg.twist.twist.linear.x))
                self.latest[source+"_w"] = (stamp, float(msg.twist.twist.angular.z))

    rclpy.init(args=ros_args)
    node = Recorder()
    try:
        if live_plot is None:
            rclpy.spin(node)
        else:
            next_draw = 0.0
            while rclpy.ok() and plt.fignum_exists(live_plot.fig.number):
                # Drain ROS callbacks in short batches, then service the GUI.
                # Both event loops stay on this thread; no shared-state thread race.
                end = time.monotonic() + 0.02
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.002)
                if time.monotonic() >= next_draw:
                    live_plot.update(node.records, node.latest)
                    next_draw = time.monotonic() + 1.0/args.refresh_hz
                plt.pause(0.001)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if live_plot is not None:
            plt.close(live_plot.fig)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    try:
        metrics = analyse(node.records, args.output)
        print(json.dumps(metrics, indent=2))
        print("Saved comparison files in", Path(args.output).resolve())
    except ValueError as error:
        print("Comparison could not be completed:", error)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
