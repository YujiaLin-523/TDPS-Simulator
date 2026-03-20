set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${ROOT_DIR}/TDPS-Simulator"
DEFAULT_SCENARIO_CONFIG="${SIM_ROOT}/sim_tests/line_follow_v1/config/scenarios_default.csv"

DURATION_SEC="${1:-15}"
DT_SEC="${2:-0.01}"
LINE_THRESHOLD="${3:-0.12}"
REPORT_PATH="${4:-${SIM_ROOT}/artifacts/line_follow_v1/reports/single_run/last_autotest_report.json}"
BASE_SEED="${5:-20260319}"
SCENARIO_CONFIG="${6:-}"
BASELINE_REPORT="${7:-}"
DISTURBANCE_PROFILE="${8:-}"

mkdir -p "$(dirname "${REPORT_PATH}")"

RUNNER_BIN="$(bash "${SCRIPT_DIR}/build_line_follow_runner.sh" | tail -n 1)"

echo "[autotest] running quick (duration=${DURATION_SEC}s, dt=${DT_SEC}s, threshold=${LINE_THRESHOLD}, seed=${BASE_SEED}, profile=${DISTURBANCE_PROFILE:-normal})..."

CMD=("${RUNNER_BIN}" quick "${DURATION_SEC}" "${DT_SEC}" "${LINE_THRESHOLD}" "${REPORT_PATH}" "${BASE_SEED}")
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

echo "[autotest] report: ${REPORT_PATH}"
exit ${EXIT_CODE}
