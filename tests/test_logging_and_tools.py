"""Tests for run logging and the analysis/comparison tooling."""

import csv
import importlib.util
import os

import pytest

from beetlebot_risk_nav.core.logging_utils import FIELDS, NavigationLogger
from beetlebot_risk_nav.core.navigator import Navigator
from beetlebot_risk_nav.core.params import LoggingConfig
from synthetic import corridor, make_scan

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'scripts')


def load_script(name):
    path = os.path.join(SCRIPTS, name)
    spec = importlib.util.spec_from_file_location(name.replace('.py', ''), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_and_log(tmp_path, cycles=15):
    config = LoggingConfig()
    config.directory = str(tmp_path)
    logger = NavigationLogger(config)
    navigator = Navigator()
    navigator.set_goal((2.0, 0.0), 0.0)
    path = logger.start_episode((2.0, 0.0), 0.0)
    x = 0.0
    for k in range(cycles):
        now = k * 0.1
        scan = make_scan(stamp=now, walls=corridor(1.5))
        result = navigator.update(scan, (x, 0.0, 0.0), (0.1, 0.0), now, odom_stamp=now)
        logger.log_cycle(now, result, (x, 0.0, 0.0))
        x += 0.02
    logger.close()
    return path


def test_logger_writes_a_complete_csv(tmp_path):
    path = run_and_log(tmp_path)
    assert path and os.path.exists(path)
    with open(path, newline='') as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert set(rows[0]) == set(FIELDS)
    assert all(row['state'] for row in rows)


def test_logging_can_be_disabled(tmp_path):
    config = LoggingConfig()
    config.enabled = False
    config.directory = str(tmp_path)
    logger = NavigationLogger(config)
    assert logger.start_episode((1.0, 0.0), 0.0) is None
    assert not logger.active


def test_logging_failure_never_breaks_the_control_loop(tmp_path):
    """A full disk must not stop the robot navigating."""
    config = LoggingConfig()
    config.directory = os.path.join(str(tmp_path), 'file_in_the_way', 'nested')
    open(os.path.join(str(tmp_path), 'blocker'), 'w').close()
    config.directory = os.path.join(str(tmp_path), 'blocker', 'nested')
    logger = NavigationLogger(config)
    assert logger.start_episode((1.0, 0.0), 0.0) is None
    assert logger.disabled_reason
    logger.log_cycle(0.0, _DummyResult(), (0.0, 0.0, 0.0))     # must not raise


def test_infinite_values_are_written_as_blanks_not_inf(tmp_path):
    """"inf" in a CSV breaks most readers; a blank cell does not."""
    config = LoggingConfig()
    config.directory = str(tmp_path)
    logger = NavigationLogger(config)
    logger.start_episode((1.0, 0.0), 0.0)
    logger.log_cycle(0.0, _DummyResult(), (0.0, 0.0, 0.0))
    logger.close()
    with open(logger.path, newline='') as handle:
        row = list(csv.DictReader(handle))[0]
    assert row['distance_to_goal'] == ''


class _DummyResult:
    """A NavResult-shaped object with no plan, as produced in IDLE."""

    def __init__(self):
        from beetlebot_risk_nav.core.types import Command
        self.command = Command(0.0, 0.0)
        self.state = 'IDLE'
        self.status = 'idle'
        self.distance_to_goal = float('inf')
        self.plan = None
        self.safety = None
        self.succeeded = False


# ----------------------------------------------------------------------
def test_analyze_log_summarises_a_real_run(tmp_path, capsys):
    path = run_and_log(tmp_path)
    analyze = load_script('analyze_log.py')
    summary = analyze.summarise(path)
    assert summary['cycles'] > 0
    assert summary['duration'] > 0.0
    assert summary['min_clearance'] is not None
    assert set(summary['risk_counts']) == set(analyze.RISK_LEVELS)
    assert analyze.main([path]) == 0
    assert 'Minimum clearance' in capsys.readouterr().out


def test_analyze_log_handles_an_empty_log(tmp_path):
    path = os.path.join(str(tmp_path), 'empty.csv')
    open(path, 'w').close()
    analyze = load_script('analyze_log.py')
    assert 'error' in analyze.summarise(path)


def test_baseline_comparison_runs_and_reports_clearances(capsys):
    compare = load_script('compare_baseline.py')
    assert compare.main([]) == 0
    output = capsys.readouterr().out
    assert 'baseline' in output
    assert 'new system' in output
    # The scope caveat must always be printed with the numbers.
    assert 'physical testing' in output.lower()


def test_baseline_port_reproduces_the_original_thresholds():
    """The comparison is only meaningful if the port is faithful."""
    from beetlebot_risk_nav.baseline.baseline_controller import BaselineAvoidanceController
    baseline = BaselineAvoidanceController()
    assert baseline.safe_distance == pytest.approx(0.5)
    assert baseline.front_distance == pytest.approx(0.45)
    assert baseline.move_speed == pytest.approx(0.2)
    assert baseline.reverse_speed == pytest.approx(0.12)
    assert baseline.turn_speed == pytest.approx(0.4)
    assert baseline.reverse_time == pytest.approx(0.8)


def test_original_baseline_script_is_preserved_unmodified():
    """The original ROS script must never be edited - it is the fallback."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, 'baseline', 'lyra_control', 'obstacle_avoidance.py')
    source = open(path).read()
    assert 'class ObstacleAvoidanceNode' in source
    assert "'/cmd_vel_nav'" in source
    assert 'self.safe_distance = 0.5' in source
    assert 'self.front_distance = 0.45' in source
