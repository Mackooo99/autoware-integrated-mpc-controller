#!/usr/bin/env python3
"""Publish minimal valid synthetic inputs to a standalone integrated_mpc_node.

Lets the controller's real onTimer()/OSQP path run and publish control_cmd
without the full Planning Simulator / Scenario Simulator stack, so callback
frequency and solver behaviour can be measured while that stack is blocked.
This is a bench/measurement aid, not a substitute for closed-loop validation.

Usage:
  ros2 run autoware_integrated_mpc_trajectory_follower synthetic_input_publisher.py \
    --ros-args -p ns:=/control/trajectory_follower/controller_node_exe
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry


def make_straight_trajectory(num_points=60, spacing=1.0, speed=5.0):
    traj = Trajectory()
    traj.header.frame_id = "map"
    for i in range(num_points):
        p = TrajectoryPoint()
        p.pose.position.x = float(i) * spacing
        p.pose.position.y = 0.0
        p.pose.orientation.w = 1.0
        p.longitudinal_velocity_mps = speed
        p.acceleration_mps2 = 0.0
        traj.points.append(p)
    return traj


class SyntheticInputPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_bench_publisher")
        self.declare_parameter("ns", "/control/trajectory_follower/controller_node_exe")
        ns = self.get_parameter("ns").value

        reliable = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        reliable_tl = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.traj_pub = self.create_publisher(Trajectory, f"{ns}/input/reference_trajectory", reliable)
        self.odom_pub = self.create_publisher(Odometry, f"{ns}/input/current_odometry", reliable)
        self.steer_pub = self.create_publisher(SteeringReport, f"{ns}/input/current_steering", reliable)
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, f"{ns}/input/current_accel", reliable
        )
        self.mode_pub = self.create_publisher(
            OperationModeState, f"{ns}/input/current_operation_mode", reliable_tl
        )

        self.trajectory = make_straight_trajectory()
        self.timer = self.create_timer(0.02, self._tick)

    def _tick(self):
        stamp = self.get_clock().now().to_msg()

        self.trajectory.header.stamp = stamp
        self.traj_pub.publish(self.trajectory)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 4.5
        self.odom_pub.publish(odom)

        steer = SteeringReport()
        steer.stamp = stamp
        steer.steering_tire_angle = 0.0
        self.steer_pub.publish(steer)

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = stamp
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear.x = 0.0
        self.accel_pub.publish(accel)

        mode = OperationModeState()
        mode.stamp = stamp
        mode.mode = OperationModeState.AUTONOMOUS
        mode.is_autoware_control_enabled = True
        self.mode_pub.publish(mode)


def main():
    rclpy.init()
    node = SyntheticInputPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
