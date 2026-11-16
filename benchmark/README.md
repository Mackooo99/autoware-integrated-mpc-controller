# Autoware Control Benchmark Monitor

Ready-to-use monitoring package for:

`~/control_benchmark`

## Copy into the project

Copy:
- `monitor_ws/` -> `~/control_benchmark/monitor_ws/`
- files from `scripts/` -> `~/control_benchmark/scripts/`

Keep the existing `~/control_benchmark/scripts/run_scenario.sh`.

## Install Python dependencies

```bash
sudo apt update
sudo apt install -y python3-numpy python3-matplotlib
```

## Build the ROS 2 monitor package

```bash
source /opt/ros/humble/setup.bash
source ~/autoware-new/Autoware/autoware/install/setup.bash

cd ~/control_benchmark/monitor_ws
colcon build --symlink-install
source install/setup.bash
```

## Make scripts executable

```bash
chmod +x ~/control_benchmark/scripts/run_experiment.sh
chmod +x ~/control_benchmark/scripts/run_suite.sh
chmod +x ~/control_benchmark/scripts/analyze_run.py
chmod +x ~/control_benchmark/scripts/compare_runs.py
```

## Smoke-test the logger

Start one scenario normally, then in another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/autoware-new/Autoware/autoware/install/setup.bash
source ~/control_benchmark/monitor_ws/install/setup.bash

ros2 run control_benchmark_monitor logger_node \
  --ros-args \
  --params-file ~/control_benchmark/monitor_ws/install/control_benchmark_monitor/share/control_benchmark_monitor/config/benchmark.yaml \
  -p output_dir:=/tmp/control_benchmark_test
```

Stop with Ctrl+C:

```bash
ls -lh /tmp/control_benchmark_test
head /tmp/control_benchmark_test/timeseries.csv
```

## Run one complete benchmark

```bash
cd ~/control_benchmark
./scripts/run_experiment.sh baseline_stock 1 1
```

With rosbag:

```bash
./scripts/run_experiment.sh baseline_stock 1 1 --bag
```

## Run all five scenarios

One repetition:

```bash
./scripts/run_suite.sh baseline_stock 1
```

Five repetitions:

```bash
./scripts/run_suite.sh baseline_stock 5
```

## Result structure

```text
results/baseline_stock/S01/run_01/
├── timeseries.csv
├── timings.csv
├── events.csv
├── reference_trajectory.csv
├── scenario.log
├── logger.log
├── metadata.yaml
├── status.txt
├── summary.json
├── summary.md
└── plots/
```

## Compare baseline and integrated MPC later

```bash
mkdir -p ~/control_benchmark/results/comparison

python3 ~/control_benchmark/scripts/compare_runs.py \
  ~/control_benchmark/results/baseline_stock \
  ~/control_benchmark/results/integrated_mpc \
  ~/control_benchmark/results/comparison
```

