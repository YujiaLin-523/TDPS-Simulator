set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${ROOT_DIR}/TDPS-Simulator"
DEFAULT_SCENARIO_CONFIG="${SIM_ROOT}/sim_tests/line_follow_v1/config/scenarios_default.csv"

DURATION_SEC="${1:-15}"
DT_SEC="${2:-0.01}"
LINE_THRESHOLD="${3:-0.12}"
SEED_START="${4:-20260319}"
RUNS="${5:-8}"
REPORT_DIR="${6:-${SIM_ROOT}/artifacts/line_follow_v1/reports/stability_runs}"
SCENARIO_CONFIG="${7:-}"
BASELINE_REPORT="${8:-}"
DISTURBANCE_PROFILE="${9:-normal}"

mkdir -p "${REPORT_DIR}"
RUNNER_BIN="$(bash "${SCRIPT_DIR}/build_line_follow_runner.sh" | tail -n 1)"

CMD=("${RUNNER_BIN}" stability "${DURATION_SEC}" "${DT_SEC}" "${LINE_THRESHOLD}" "${SEED_START}" "${RUNS}" "${REPORT_DIR}")
if [[ -n "${SCENARIO_CONFIG}" ]]; then
  CMD+=("${SCENARIO_CONFIG}")
fi
if [[ -n "${BASELINE_REPORT}" ]]; then
  if [[ -z "${SCENARIO_CONFIG}" ]]; then
    CMD+=("${DEFAULT_SCENARIO_CONFIG}")
  fi
  CMD+=("${BASELINE_REPORT}")
fi
if [[ -n "${DISTURBANCE_PROFILE}" ]]; then
  CMD+=("${DISTURBANCE_PROFILE}")
fi

set +e
"${CMD[@]}"
EXIT_CODE=$?
set -e

exit ${EXIT_CODE}
