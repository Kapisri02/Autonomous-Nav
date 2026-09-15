#!/usr/bin/env bash
# Full software validation: syntax/build check, lint, then the test suite.
#
# This is the "build" step for a pure-Python ROS package: there is nothing to
# compile, so byte-compilation plus an import of every module is the equivalent
# check that the code is well-formed and loadable.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_ROOT="${REPO_ROOT}/src/beetlebot_risk_nav"
export PYTHONPATH="${PACKAGE_ROOT}:${REPO_ROOT}/tests:${PYTHONPATH:-}"

PYTHON="${PYTHON:-python3}"
PYTEST="${PYTEST:-$(command -v pytest || echo "${PYTHON} -m pytest")}"
FLAKE8="${FLAKE8:-$(command -v flake8 || true)}"

status=0
step() { printf '\n=== %s ===\n' "$1"; }
fail() { echo "FAILED: $1"; status=1; }

step "1/4 byte-compile (build check)"
"${PYTHON}" -m compileall -q "${PACKAGE_ROOT}" "${REPO_ROOT}/tests" \
    "${REPO_ROOT}/baseline" > /dev/null && echo "all modules compile" || fail "compileall"

step "2/4 import check"
"${PYTHON}" - <<'PY' || fail "imports"
import importlib
import pkgutil

import beetlebot_risk_nav

count = 0
for module in pkgutil.walk_packages(beetlebot_risk_nav.__path__,
                                    beetlebot_risk_nav.__name__ + '.'):
    name = module.name
    if '.nodes.' in name:
        continue          # ROS nodes need rclpy; checked separately
    importlib.import_module(name)
    count += 1
print('imported {} modules without ROS'.format(count))
PY

step "3/4 lint"
if [ -n "${FLAKE8}" ]; then
    "${FLAKE8}" "${PACKAGE_ROOT}/beetlebot_risk_nav" "${REPO_ROOT}/tests" \
        && echo "lint clean" || fail "flake8"
else
    echo "flake8 not installed - skipped"
fi

step "4/4 tests"
${PYTEST} "${REPO_ROOT}/tests" -q || fail "pytest"

printf '\n==================================\n'
if [ "${status}" -eq 0 ]; then
    echo "ALL SOFTWARE CHECKS PASSED"
else
    echo "SOFTWARE CHECKS FAILED"
fi
echo "Note: software tests only. Physical robot behaviour is NOT validated here."
exit "${status}"
