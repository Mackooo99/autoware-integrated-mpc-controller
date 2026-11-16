#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <controller_name> [repetitions] [--bag]"
  echo "Example: $0 baseline_stock 5"
  exit 2
fi

CONTROLLER="$1"
REPETITIONS="${2:-1}"
BAG_MODE="${3:-}"
ROOT="${HOME}/control_benchmark"

for RUN in $(seq 1 "${REPETITIONS}"); do
  for SCENARIO in 1 2 3 4 5; do
    echo
    echo "===== ${CONTROLLER}: S$(printf "%02d" "${SCENARIO}") run ${RUN}/${REPETITIONS} ====="
    if [[ "${BAG_MODE}" == "--bag" ]]; then
      "${ROOT}/scripts/run_experiment.sh" "${CONTROLLER}" "${SCENARIO}" "${RUN}" --bag
    else
      "${ROOT}/scripts/run_experiment.sh" "${CONTROLLER}" "${SCENARIO}" "${RUN}"
    fi
  done
done

