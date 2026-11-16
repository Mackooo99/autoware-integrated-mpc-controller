#!/usr/bin/env bash

set -e

# ============================================================
# Autoware Controller Benchmark - Scenario Runner
# ============================================================

AUTOWARE_WS="$HOME/autoware-new/Autoware/autoware"
SCENARIO_DIR="$HOME/control_benchmark/scenarios"

ARCHITECTURE="awf/universe/20250130"
VEHICLE_MODEL="sample_vehicle"
SENSOR_MODEL="sample_sensor_kit"

# ------------------------------------------------------------
# Check arguments
# ------------------------------------------------------------

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo ""
    echo "Usage:"
    echo "  $0 <scenario_number> [controller]"
    echo ""
    echo "Available scenarios:"
    echo "  1 - Straight longitudinal 0-20 km/h"
    echo "  2 - Single intersection turn"
    echo "  3 - Alternating opposite turns"
    echo "  4 - Continuous curved-loop traversal"
    echo "  5 - 180-degree U-turn"
    echo ""
    echo "controller (default: integrated_mpc_v1):"
    echo "  baseline_stock    - stock Autoware separated lateral MPC + longitudinal PID"
    echo "  integrated_mpc_v1 - custom integrated lateral-longitudinal MPC"
    echo ""
    exit 1
fi

CONTROLLER="${2:-integrated_mpc_v1}"

case "$CONTROLLER" in
    baseline_stock)
        AUTOWARE_LAUNCH_PACKAGE="autoware_launch"
        AUTOWARE_LAUNCH_FILE="planning_simulator.launch.xml"
        ;;
    integrated_mpc_v1)
        AUTOWARE_LAUNCH_PACKAGE="autoware_integrated_mpc_trajectory_follower"
        AUTOWARE_LAUNCH_FILE="integrated_planning_simulator.launch.xml"
        ;;
    *)
        echo "[ERROR] Unknown controller: $CONTROLLER"
        echo "Valid values are: baseline_stock, integrated_mpc_v1"
        exit 1
        ;;
esac

# ------------------------------------------------------------
# Select scenario
# ------------------------------------------------------------

case "$1" in

    1)
        SCENARIO_NAME="S01_Longitudinal_0to20"
        SCENARIO="$SCENARIO_DIR/S01_Longitudinal_0to20.yaml"
        ;;

    2)
        SCENARIO_NAME="S02_Single_Intersection_Turn"
        SCENARIO="$SCENARIO_DIR/S02_Single_Intersection_Turn.yaml"
        ;;

    3)
        SCENARIO_NAME="S03_Alternating_Turns"
        SCENARIO="$SCENARIO_DIR/S03_Alternating_Turns.yaml"
        ;;

    4)
        SCENARIO_NAME="S04_Continuous_U_Curved_Loop"
        SCENARIO="$SCENARIO_DIR/S04_Continuous_U_Curved_Loop.yaml"
        ;;

    5)
        SCENARIO_NAME="S05_Continuous_180_Curved_Loop"
        SCENARIO="$SCENARIO_DIR/S05_Continuous_180_Curved_Loop.yaml"
        ;;

    *)
        echo "[ERROR] Unknown scenario: $1"
        echo "Valid values are: 1, 2, 3, 4, 5"
        exit 1
        ;;
esac

# ------------------------------------------------------------
# Check paths
# ------------------------------------------------------------

if [ ! -d "$AUTOWARE_WS" ]; then
    echo "[ERROR] Autoware workspace not found:"
    echo "        $AUTOWARE_WS"
    exit 1
fi

if [ ! -f "$AUTOWARE_WS/install/setup.bash" ]; then
    echo "[ERROR] Autoware install/setup.bash not found."
    echo "        Has the workspace been built?"
    exit 1
fi

if [ ! -f "$SCENARIO" ]; then
    echo "[ERROR] Scenario file not found:"
    echo "        $SCENARIO"
    exit 1
fi

# ------------------------------------------------------------
# ROS / Autoware environment
# ------------------------------------------------------------

source /opt/ros/humble/setup.bash
source "$AUTOWARE_WS/install/setup.bash"
source "$HOME/integrated_mpc_ws/install/setup.bash"

# ------------------------------------------------------------
# Information
# ------------------------------------------------------------

echo ""
echo "============================================================"
echo " AUTOWARE CONTROLLER BENCHMARK"
echo "============================================================"
echo " Scenario   : $1"
echo " Name       : $SCENARIO_NAME"
echo " YAML       : $SCENARIO"
echo " Controller : $CONTROLLER"
echo " Launch     : $AUTOWARE_LAUNCH_PACKAGE/$AUTOWARE_LAUNCH_FILE"
echo " Autoware   : $AUTOWARE_WS"
echo " Profile    : $ARCHITECTURE"
echo " Vehicle    : $VEHICLE_MODEL"
echo " Sensor     : $SENSOR_MODEL"
echo "============================================================"
echo ""

# ------------------------------------------------------------
# Run Scenario Simulator + Autoware planning simulation
# ------------------------------------------------------------

exec ros2 launch scenario_test_runner scenario_test_runner.launch.py \
    architecture_type:="$ARCHITECTURE" \
    scenario:="$SCENARIO" \
    vehicle_model:="$VEHICLE_MODEL" \
    sensor_model:="$SENSOR_MODEL" \
    record:=false \
    autoware_launch_package:="$AUTOWARE_LAUNCH_PACKAGE" \
    autoware_launch_file:="$AUTOWARE_LAUNCH_FILE"
