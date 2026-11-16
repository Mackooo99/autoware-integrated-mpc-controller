#!/usr/bin/env bash

set -e

AUTOWARE_WS="$HOME/autoware-new/Autoware/autoware"

ARCHITECTURE="awf/universe/20250130"
VEHICLE_MODEL="sample_vehicle"
SENSOR_MODEL="sample_sensor_kit"

if [ "$#" -lt 1 ]; then
    echo "Usage:"
    echo "  $0 <scenario.yaml>"
    echo
    echo "Example:"
    echo "  $0 ~/scenarios/S01_Straight_Longitudinal.yaml"
    exit 1
fi

SCENARIO="$1"

# Convert to absolute path
SCENARIO="$(realpath "$SCENARIO")"

if [ ! -f "$SCENARIO" ]; then
    echo "[ERROR] Scenario does not exist:"
    echo "        $SCENARIO"
    exit 1
fi

echo "=============================================="
echo " AUTOWARE SCENARIO TEST"
echo "=============================================="
echo "Workspace:     $AUTOWARE_WS"
echo "Scenario:      $SCENARIO"
echo "Architecture:  $ARCHITECTURE"
echo "Vehicle:       $VEHICLE_MODEL"
echo "Sensor kit:    $SENSOR_MODEL"
echo "Simulator:     Planning / Scenario Simulator"
echo "=============================================="

source /opt/ros/humble/setup.bash
source "$AUTOWARE_WS/install/setup.bash"

exec ros2 launch scenario_test_runner scenario_test_runner.launch.py \
    architecture_type:="$ARCHITECTURE" \
    record:=false \
    scenario:="$SCENARIO" \
    sensor_model:="$SENSOR_MODEL" \
    vehicle_model:="$VEHICLE_MODEL"
