set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${ROOT_DIR}/TDPS-Simulator"
SIM_TEST_DIR="${SIM_ROOT}/sim_tests"
LF_COMMON_DIR="${SIM_TEST_DIR}/common"
LF_SUITE_DIR="${SIM_TEST_DIR}/line_follow_v1"
ARTIFACT_DIR="${SIM_ROOT}/artifacts/line_follow_v1"
RUNNER_BIN="${ARTIFACT_DIR}/bin/lf_autotest_runner"

mkdir -p "$(dirname "${RUNNER_BIN}")"

echo "[build] compiling line-follow autotest runner..."
gcc -std=c11 -Wall -Wextra -Werror \
  -I"${ROOT_DIR}/code/line_follow_v1/Inc" \
  -I"${LF_COMMON_DIR}" \
  -I"${LF_COMMON_DIR}/harness" \
  -I"${LF_SUITE_DIR}" \
  "${ROOT_DIR}/code/line_follow_v1/Src/lf_app.c" \
  "${ROOT_DIR}/code/line_follow_v1/Src/lf_chassis.c" \
  "${ROOT_DIR}/code/line_follow_v1/Src/lf_config.c" \
  "${ROOT_DIR}/code/line_follow_v1/Src/lf_control.c" \
  "${ROOT_DIR}/code/line_follow_v1/Src/lf_sensor.c" \
  "${ROOT_DIR}/code/line_follow_v1/Src/lf_future_hooks.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_core.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_scenarios.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_evaluator.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_report.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_baseline.c" \
  "${LF_COMMON_DIR}/harness/lf_harness_cli.c" \
  "${LF_SUITE_DIR}/lf_autotest_harness.c" \
  -lm -o "${RUNNER_BIN}"

echo "${RUNNER_BIN}"
