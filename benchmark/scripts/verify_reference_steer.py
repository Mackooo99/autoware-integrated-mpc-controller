#!/usr/bin/env python3

import csv
import math
import sys
from pathlib import Path

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TRAJECTORY_TOPIC = "/planning/trajectory"
ODOM_TOPIC = "/localization/kinematic_state"
CONTROL_TOPIC = "/control/trajectory_follower/control_cmd"
STEERING_TOPIC = "/vehicle/status/steering_status"
BOUNDARY_TOPIC = "/diagnostics/motion_velocity_planner/boundary_departure"

WHEELBASE = 2.79


def signed_curvature(p0, p1, p2):
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2

    a = math.hypot(x1 - x0, y1 - y0)
    b = math.hypot(x2 - x1, y2 - y1)
    c = math.hypot(x2 - x0, y2 - y0)

    denom = a * b * c

    if denom < 1e-9:
        return float("nan")

    cross = (
        (x1 - x0) * (y2 - y0)
        - (y1 - y0) * (x2 - x0)
    )

    return 2.0 * cross / denom


def nearest_trajectory_point(traj, x, y):
    if traj is None or len(traj.points) == 0:
        return None, None

    best_i = None
    best_d2 = float("inf")

    for i, p in enumerate(traj.points):
        dx = x - p.pose.position.x
        dy = y - p.pose.position.y
        d2 = dx * dx + dy * dy

        if d2 < best_d2:
            best_d2 = d2
            best_i = i

    return best_i, traj.points[best_i]


def curvature_at(traj, i):
    if traj is None:
        return float("nan")

    if i is None or i <= 0 or i >= len(traj.points) - 1:
        return float("nan")

    p0 = traj.points[i - 1].pose.position
    p1 = traj.points[i].pose.position
    p2 = traj.points[i + 1].pose.position

    return signed_curvature(
        (p0.x, p0.y),
        (p1.x, p1.y),
        (p2.x, p2.y),
    )


def finite_array(values):
    x = np.asarray(values, dtype=float)
    return x[np.isfinite(x)]


def main():

    if len(sys.argv) != 3:
        print(
            "Usage: verify_reference_steer.py "
            "<bag_directory> <output_csv>"
        )
        return 2

    bag_dir = Path(sys.argv[1]).resolve()
    output_csv = Path(sys.argv[2]).resolve()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id="sqlite3",
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    required_topics = [
        TRAJECTORY_TOPIC,
        ODOM_TOPIC,
        CONTROL_TOPIC,
        STEERING_TOPIC,
        BOUNDARY_TOPIC,
    ]

    print("\nAvailable required topics:")

    for topic in required_topics:
        print(
            f"{topic:70s}",
            "YES" if topic in topic_types else "NO",
        )

    msg_types = {
        topic: get_message(msg_type)
        for topic, msg_type in topic_types.items()
    }

    latest_traj = None
    latest_controller_steer = float("nan")
    latest_actual_steer = float("nan")

    first_bag_time_ns = None

    rows = []
    boundary_events = []

    while reader.has_next():

        topic, data, timestamp_ns = reader.read_next()

        if first_bag_time_ns is None:
            first_bag_time_ns = timestamp_ns

        t = (timestamp_ns - first_bag_time_ns) * 1e-9

        if topic not in msg_types:
            continue

        msg = deserialize_message(
            data,
            msg_types[topic],
        )

        if topic == TRAJECTORY_TOPIC:
            latest_traj = msg
            continue

        if topic == CONTROL_TOPIC:
            latest_controller_steer = float(
                msg.lateral.steering_tire_angle
            )
            continue

        if topic == STEERING_TOPIC:
            latest_actual_steer = float(
                msg.steering_tire_angle
            )
            continue

        if topic == BOUNDARY_TOPIC:
            boundary_events.append(
                (
                    t,
                    msg.value,
                )
            )
            continue

        if topic != ODOM_TOPIC:
            continue

        if latest_traj is None:
            continue

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x

        i, p = nearest_trajectory_point(
            latest_traj,
            x,
            y,
        )

        if p is None:
            continue

        traj_front_steer = float(
            p.front_wheel_angle_rad
        )

        curvature = curvature_at(
            latest_traj,
            i,
        )

        curvature_steer = (
            math.atan(WHEELBASE * curvature)
            if math.isfinite(curvature)
            else float("nan")
        )

        rows.append(
            {
                "time_s": t,
                "x": x,
                "y": y,
                "velocity": v,
                "trajectory_index": i,

                "trajectory_front_wheel_angle_rad":
                    traj_front_steer,

                "curvature":
                    curvature,

                "curvature_required_steer_rad":
                    curvature_steer,

                "controller_steer_rad":
                    latest_controller_steer,

                "actual_steer_rad":
                    latest_actual_steer,
            }
        )

    if not rows:
        print("\nNo synchronized trajectory/odometry samples.")
        return 1

    output_csv.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    fields = list(rows[0].keys())

    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=fields,
        )
        writer.writeheader()
        writer.writerows(rows)

    # --------------------------------------------------
    # Statistics
    # --------------------------------------------------

    traj_ref = np.array(
        [
            r["trajectory_front_wheel_angle_rad"]
            for r in rows
        ],
        dtype=float,
    )

    curvature_ref = np.array(
        [
            r["curvature_required_steer_rad"]
            for r in rows
        ],
        dtype=float,
    )

    controller = np.array(
        [
            r["controller_steer_rad"]
            for r in rows
        ],
        dtype=float,
    )

    actual = np.array(
        [
            r["actual_steer_rad"]
            for r in rows
        ],
        dtype=float,
    )

    times = np.array(
        [
            r["time_s"]
            for r in rows
        ],
        dtype=float,
    )

    print("\n")
    print("=" * 75)
    print("REFERENCE STEERING VERIFICATION")
    print("=" * 75)

    print("Samples:", len(rows))

    def max_abs(name, x):
        valid = np.isfinite(x)

        if not np.any(valid):
            print(f"{name}: no data")
            return

        idx_candidates = np.where(valid)[0]
        local = np.argmax(
            np.abs(x[valid])
        )
        idx = idx_candidates[local]

        print(
            f"{name:40s}: "
            f"{x[idx]: .6f} rad "
            f"at t={times[idx]:.3f} s"
        )

    max_abs(
        "Trajectory front_wheel_angle",
        traj_ref,
    )

    max_abs(
        "Curvature-derived steering",
        curvature_ref,
    )

    max_abs(
        "Controller steering",
        controller,
    )

    max_abs(
        "Actual steering",
        actual,
    )

    valid = (
        np.isfinite(traj_ref)
        & np.isfinite(curvature_ref)
    )

    if np.any(valid):
        diff = (
            traj_ref[valid]
            - curvature_ref[valid]
        )

        print(
            "\nRMSE("
            "trajectory_front_wheel - curvature_steer"
            f") = {np.sqrt(np.mean(diff**2)):.6f} rad"
        )

        print(
            "Max |difference| = "
            f"{np.max(np.abs(diff)):.6f} rad"
        )

    valid = (
        np.isfinite(traj_ref)
        & np.isfinite(controller)
    )

    if np.any(valid):
        diff = (
            traj_ref[valid]
            - controller[valid]
        )

        print(
            "\nRMSE("
            "trajectory_front_wheel - controller"
            f") = {np.sqrt(np.mean(diff**2)):.6f} rad"
        )

    valid = (
        np.isfinite(controller)
        & np.isfinite(actual)
    )

    if np.any(valid):
        diff = (
            controller[valid]
            - actual[valid]
        )

        print(
            "RMSE(controller - actual)"
            f" = {np.sqrt(np.mean(diff**2)):.6f} rad"
        )

    # --------------------------------------------------
    # Print critical samples
    # --------------------------------------------------

    print("\n")
    print("=" * 75)
    print("SAMPLES AT MAXIMUM GEOMETRIC STEERING")
    print("=" * 75)

    valid = np.isfinite(curvature_ref)

    if np.any(valid):
        idx_candidates = np.where(valid)[0]
        local = np.argmax(
            np.abs(curvature_ref[valid])
        )
        idx = idx_candidates[local]

        for key, value in rows[idx].items():
            print(
                f"{key:40s}: {value}"
            )

    print("\n")
    print("=" * 75)
    print("BOUNDARY EVENTS")
    print("=" * 75)

    for t, value in boundary_events:
        print(
            f"t={t:8.3f} s   state={value}"
        )

    print("\nCSV saved to:")
    print(output_csv)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
