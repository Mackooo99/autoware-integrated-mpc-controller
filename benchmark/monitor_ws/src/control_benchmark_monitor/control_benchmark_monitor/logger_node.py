#!/usr/bin/env python3
import csv
import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_internal_debug_msgs.msg import Float64Stamped
from tier4_simulation_msgs.msg import UserDefinedValue


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def parse_user_value(msg: UserDefinedValue) -> float:
    try:
        return float(msg.value)
    except (ValueError, TypeError):
        return float("nan")


def signed_curvature(p0, p1, p2) -> float:
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    a = math.hypot(x1 - x0, y1 - y0)
    b = math.hypot(x2 - x1, y2 - y1)
    c = math.hypot(x2 - x0, y2 - y0)
    denom = a * b * c
    if denom < 1e-9:
        return 0.0
    cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    return 2.0 * cross / denom


class BenchmarkLogger(Node):
    def __init__(self):
        super().__init__("benchmark_logger")

        self.declare_parameter("output_dir", "/tmp/control_benchmark_run")
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("wheelbase_m", 2.79)
        self.declare_parameter("scenario_max_steer_rad", 0.5236)
        self.declare_parameter("autoware_max_steer_rad", 0.7)
        self.declare_parameter("controller_name", "baseline_stock")
        self.declare_parameter("scenario_name", "UNKNOWN")
        self.declare_parameter("run_id", 1)

        self.output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.sample_rate_hz = float(
            self.get_parameter("sample_rate_hz").get_parameter_value().double_value
        )
        self.wheelbase_m = float(
            self.get_parameter("wheelbase_m").get_parameter_value().double_value
        )
        self.scenario_max_steer_rad = float(
            self.get_parameter("scenario_max_steer_rad").get_parameter_value().double_value
        )
        self.autoware_max_steer_rad = float(
            self.get_parameter("autoware_max_steer_rad").get_parameter_value().double_value
        )
        self.controller_name = self.get_parameter("controller_name").get_parameter_value().string_value
        self.scenario_name = self.get_parameter("scenario_name").get_parameter_value().string_value
        self.run_id = self.get_parameter("run_id").get_parameter_value().integer_value

        self.start_ns = self.get_clock().now().nanoseconds
        self.closed = False

        self.odom: Optional[Odometry] = None
        self.accel: Optional[AccelWithCovarianceStamped] = None
        self.steering: Optional[SteeringReport] = None
        self.trajectory: Optional[Trajectory] = None
        self.controller_cmd: Optional[Control] = None
        self.final_cmd: Optional[Control] = None
        
        self.odom_seq = 0
        self.last_sampled_odom_seq = -1
        
        self.metrics: Dict[str, float] = {}
        self.boundary_raw = ""
        self.last_boundary_raw = None

        self.qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.timeseries_fields = [
            "time_s", "ros_time_s", "odom_stamp_s",
            "x", "y", "z", "yaw",
            "velocity_actual", "acceleration_actual", "yaw_rate",
            "ref_x", "ref_y", "ref_yaw", "ref_velocity",
            "ref_curvature", "required_steer",
            "velocity_error_raw", "lateral_error_raw", "yaw_error_raw",
            "velocity_error_eval", "lateral_error_eval", "yaw_error_eval",
            "velocity_eval", "acceleration_eval", "jerk_eval",
            "lateral_acceleration_abs",
            "steering_angle_eval", "steering_rate_eval", "steering_acceleration_eval",
            "steering_controller", "velocity_controller", "acceleration_controller",
            "steering_final", "velocity_final", "acceleration_final",
            "steering_actual",
            "left_boundary_distance", "right_boundary_distance",
            "left_uncrossable_boundary_distance", "right_uncrossable_boundary_distance",
            "boundary_departure_value",
        ]
        
        self.ts_file = open(self.output_dir / "timeseries.csv", "w", newline="", buffering=1)
        self.ts_writer = csv.DictWriter(self.ts_file, fieldnames=self.timeseries_fields)
        self.ts_writer.writeheader()

        self.timing_file = open(self.output_dir / "timings.csv", "w", newline="", buffering=1)
        self.timing_writer = csv.writer(self.timing_file)
        self.timing_writer.writerow(["time_s", "controller", "processing_time_ms"])

        self.event_file = open(self.output_dir / "events.csv", "w", newline="", buffering=1)
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(["time_s", "event", "value"])

        self.create_subscription(Odometry, "/localization/kinematic_state", self._odom_cb, self.qos)
        self.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self._accel_cb, self.qos)
        self.create_subscription(SteeringReport, "/vehicle/status/steering_status", self._steering_cb, self.qos)
        self.create_subscription(Trajectory, "/planning/trajectory", self._trajectory_cb, self.qos)
        self.create_subscription(Control, "/control/trajectory_follower/control_cmd", self._controller_cmd_cb, self.qos)
        self.create_subscription(Control, "/control/command/control_cmd", self._final_cmd_cb, self.qos)

        metric_topics = {
            "velocity_error_eval": "/control/control_evaluator/metrics/longitudinal_velocity_deviation",
            "lateral_error_eval": "/control/control_evaluator/metrics/lateral_deviation",
            "yaw_error_eval": "/control/control_evaluator/metrics/yaw_deviation",
            "velocity_eval": "/control/control_evaluator/metrics/velocity",
            "acceleration_eval": "/control/control_evaluator/metrics/acceleration",
            "jerk_eval": "/control/control_evaluator/metrics/jerk",
            "lateral_acceleration_abs": "/control/control_evaluator/metrics/lateral_acceleration_abs",
            "steering_angle_eval": "/control/control_evaluator/metrics/steering_angle",
            "steering_rate_eval": "/control/control_evaluator/metrics/steering_rate",
            "steering_acceleration_eval": "/control/control_evaluator/metrics/steering_acceleration",
            "left_boundary_distance": "/control/control_evaluator/metrics/left_boundary_distance",
            "right_boundary_distance": "/control/control_evaluator/metrics/right_boundary_distance",
            "left_uncrossable_boundary_distance": "/control/control_evaluator/metrics/left_uncrossable_boundary_distance",
            "right_uncrossable_boundary_distance": "/control/control_evaluator/metrics/right_uncrossable_boundary_distance",
        }
        for key, topic in metric_topics.items():
            self.create_subscription(
                UserDefinedValue, topic,
                lambda msg, metric_key=key: self._metric_cb(metric_key, msg),
                self.qos,
            )

        self.create_subscription(
            UserDefinedValue,
            "/diagnostics/motion_velocity_planner/boundary_departure",
            self._boundary_cb,
            self.qos,
        )

        self.create_subscription(
            Float64Stamped,
            "/control/trajectory_follower/controller_node_exe/lateral/debug/processing_time_ms",
            self._lateral_time_cb,
            self.qos,
        )
        self.create_subscription(
            Float64Stamped,
            "/control/trajectory_follower/controller_node_exe/longitudinal/debug/processing_time_ms",
            self._longitudinal_time_cb,
            self.qos,
        )

        # Integrated MPC (autoware_integrated_mpc_trajectory_follower) has no
        # separate lateral/longitudinal stage, so it is not published on the
        # two topics above. It publishes its own whole-callback and
        # OSQP-solver-only timings instead; logged under distinct controller
        # labels in the same timings.csv so the two controllers' timing
        # semantics are never conflated.
        self.create_subscription(
            Float64Stamped,
            "/control/trajectory_follower/controller_node_exe/debug/processing_time_ms",
            self._integrated_callback_time_cb,
            self.qos,
        )
        self.create_subscription(
            Float64Stamped,
            "/control/trajectory_follower/controller_node_exe/debug/solver_time_ms",
            self._integrated_solver_time_cb,
            self.qos,
        )

        period = 1.0 / max(self.sample_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._sample)

        self.get_logger().info(
            f"Benchmark logger started: controller={self.controller_name}, "
            f"scenario={self.scenario_name}, run={self.run_id}, output={self.output_dir}"
        )

    def _elapsed(self) -> float:
        return (self.get_clock().now().nanoseconds - self.start_ns) * 1e-9

    def _ros_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg): 
        self.odom = msg
        self.odom_seq += 1
        
    def _accel_cb(self, msg): self.accel = msg
    def _steering_cb(self, msg): self.steering = msg
    def _trajectory_cb(self, msg): self.trajectory = msg
    def _controller_cmd_cb(self, msg): self.controller_cmd = msg
    def _final_cmd_cb(self, msg): self.final_cmd = msg

    def _metric_cb(self, key: str, msg: UserDefinedValue):
        self.metrics[key] = parse_user_value(msg)

    def _boundary_cb(self, msg: UserDefinedValue):
        self.boundary_raw = msg.value
        if msg.value != self.last_boundary_raw:
            self.event_writer.writerow([f"{self._elapsed():.9f}", "boundary_departure", msg.value])
            self.last_boundary_raw = msg.value

    def _lateral_time_cb(self, msg: Float64Stamped):
        self.timing_writer.writerow([f"{self._elapsed():.9f}", "lateral", f"{msg.data:.9f}"])

    def _longitudinal_time_cb(self, msg: Float64Stamped):
        self.timing_writer.writerow([f"{self._elapsed():.9f}", "longitudinal", f"{msg.data:.9f}"])

    def _integrated_callback_time_cb(self, msg: Float64Stamped):
        self.timing_writer.writerow([f"{self._elapsed():.9f}", "integrated_callback", f"{msg.data:.9f}"])

    def _integrated_solver_time_cb(self, msg: Float64Stamped):
        self.timing_writer.writerow([f"{self._elapsed():.9f}", "integrated_solver", f"{msg.data:.9f}"])

    def _nearest_reference(
        self, x: float, y: float, yaw: float
    ) -> Tuple[float, float, float, float, float, float, float, float]:
        if self.trajectory is None or len(self.trajectory.points) == 0:
            nan = float("nan")
            return nan, nan, nan, nan, nan, nan, nan, nan

        pts = self.trajectory.points
        best_i = 0
        best_d2 = float("inf")
        for i, p in enumerate(pts):
            px = p.pose.position.x
            py = p.pose.position.y
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        p = pts[best_i]
        ref_x = p.pose.position.x
        ref_y = p.pose.position.y
        ref_yaw = quat_to_yaw(p.pose.orientation)
        ref_velocity = float(p.longitudinal_velocity_mps)

        curvature = 0.0
        if len(pts) >= 3:
            i0 = max(0, best_i - 1)
            i2 = min(len(pts) - 1, best_i + 1)
            if i0 != best_i and i2 != best_i:
                p0 = pts[i0].pose.position
                p1 = pts[best_i].pose.position
                p2 = pts[i2].pose.position
                curvature = signed_curvature((p0.x, p0.y), (p1.x, p1.y), (p2.x, p2.y))

        required_steer = math.atan(self.wheelbase_m * curvature)
        dx = x - ref_x
        dy = y - ref_y
        lateral_error_raw = -math.sin(ref_yaw) * dx + math.cos(ref_yaw) * dy
        yaw_error_raw = wrap_angle(yaw - ref_yaw)

        return (
            ref_x, ref_y, ref_yaw, ref_velocity,
            curvature, required_steer,
            lateral_error_raw, yaw_error_raw
        )

    def _sample(self):
        if self.odom is None:
            return
            
        if self.odom_seq == self.last_sampled_odom_seq:
            return

        self.last_sampled_odom_seq = self.odom_seq

        pos = self.odom.pose.pose.position
        ori = self.odom.pose.pose.orientation
        yaw = quat_to_yaw(ori)
        velocity_actual = float(self.odom.twist.twist.linear.x)
        yaw_rate = float(self.odom.twist.twist.angular.z)

        acceleration_actual = float("nan")
        if self.accel is not None:
            acceleration_actual = float(self.accel.accel.accel.linear.x)

        steering_actual = float("nan")
        if self.steering is not None:
            steering_actual = float(self.steering.steering_tire_angle)

        (
            ref_x, ref_y, ref_yaw, ref_velocity,
            ref_curvature, required_steer,
            lateral_error_raw, yaw_error_raw
        ) = self._nearest_reference(pos.x, pos.y, yaw)

        velocity_error_raw = ref_velocity - velocity_actual if math.isfinite(ref_velocity) else float("nan")

        steering_controller = velocity_controller = acceleration_controller = float("nan")
        if self.controller_cmd is not None:
            steering_controller = float(self.controller_cmd.lateral.steering_tire_angle)
            velocity_controller = float(self.controller_cmd.longitudinal.velocity)
            acceleration_controller = float(self.controller_cmd.longitudinal.acceleration)

        steering_final = velocity_final = acceleration_final = float("nan")
        if self.final_cmd is not None:
            steering_final = float(self.final_cmd.lateral.steering_tire_angle)
            velocity_final = float(self.final_cmd.longitudinal.velocity)
            acceleration_final = float(self.final_cmd.longitudinal.acceleration)

        try:
            boundary_value = float(self.boundary_raw)
        except (ValueError, TypeError):
            boundary_value = float("nan")
            
            
        odom_stamp_s = (float(self.odom.header.stamp.sec) + float(self.odom.header.stamp.nanosec) * 1e-9)

        row = {
            "time_s": self._elapsed(),
            "ros_time_s": self._ros_time(),
            "odom_stamp_s": odom_stamp_s,
            "x": pos.x, "y": pos.y, "z": pos.z, "yaw": yaw,
            "velocity_actual": velocity_actual,
            "acceleration_actual": acceleration_actual,
            "yaw_rate": yaw_rate,
            "ref_x": ref_x, "ref_y": ref_y, "ref_yaw": ref_yaw,
            "ref_velocity": ref_velocity, "ref_curvature": ref_curvature,
            "required_steer": required_steer,
            "velocity_error_raw": velocity_error_raw,
            "lateral_error_raw": lateral_error_raw,
            "yaw_error_raw": yaw_error_raw,
            "velocity_error_eval": self.metrics.get("velocity_error_eval", float("nan")),
            "lateral_error_eval": self.metrics.get("lateral_error_eval", float("nan")),
            "yaw_error_eval": self.metrics.get("yaw_error_eval", float("nan")),
            "velocity_eval": self.metrics.get("velocity_eval", float("nan")),
            "acceleration_eval": self.metrics.get("acceleration_eval", float("nan")),
            "jerk_eval": self.metrics.get("jerk_eval", float("nan")),
            "lateral_acceleration_abs": self.metrics.get("lateral_acceleration_abs", float("nan")),
            "steering_angle_eval": self.metrics.get("steering_angle_eval", float("nan")),
            "steering_rate_eval": self.metrics.get("steering_rate_eval", float("nan")),
            "steering_acceleration_eval": self.metrics.get("steering_acceleration_eval", float("nan")),
            "steering_controller": steering_controller,
            "velocity_controller": velocity_controller,
            "acceleration_controller": acceleration_controller,
            "steering_final": steering_final,
            "velocity_final": velocity_final,
            "acceleration_final": acceleration_final,
            "steering_actual": steering_actual,
            "left_boundary_distance": self.metrics.get("left_boundary_distance", float("nan")),
            "right_boundary_distance": self.metrics.get("right_boundary_distance", float("nan")),
            "left_uncrossable_boundary_distance": self.metrics.get("left_uncrossable_boundary_distance", float("nan")),
            "right_uncrossable_boundary_distance": self.metrics.get("right_uncrossable_boundary_distance", float("nan")),
            "boundary_departure_value": boundary_value,
        }
        self.ts_writer.writerow(row)

    def _write_reference_trajectory(self):
        if self.trajectory is None or len(self.trajectory.points) == 0:
            return
        path = self.output_dir / "reference_trajectory.csv"
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "index", "x", "y", "z", "yaw",
                "velocity_mps", "acceleration_mps2", "front_wheel_angle_rad"
            ])
            for i, p in enumerate(self.trajectory.points):
                writer.writerow([
                    i,
                    p.pose.position.x, p.pose.position.y, p.pose.position.z,
                    quat_to_yaw(p.pose.orientation),
                    p.longitudinal_velocity_mps,
                    p.acceleration_mps2,
                    p.front_wheel_angle_rad,
                ])

    def destroy_node(self):
        if not self.closed:
            self.closed = True
            try:
                self._write_reference_trajectory()
            except Exception as exc:
                self.get_logger().warning(f"Could not write reference trajectory: {exc}")
            for f in [self.ts_file, self.timing_file, self.event_file]:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
            self.get_logger().info(f"Benchmark data saved to {self.output_dir}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

