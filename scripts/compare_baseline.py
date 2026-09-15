#!/usr/bin/env python3
"""Compare the new navigation system against the preserved baseline.

    ./scripts/compare_baseline.py
    ./scripts/compare_baseline.py --verbose

Both controllers are shown the **same** synthetic scans and their decisions are
tabulated side by side. The baseline logic is the ROS-free port of the original
``obstacle_avoidance.py`` (thresholds and state machine unchanged); the original
ROS script itself is preserved untouched at
``baseline/lyra_control/obstacle_avoidance.py``.

Scope, stated plainly: this compares **decisions**, not outcomes. There is no
robot model here and no closed loop, so it cannot tell you which controller
travels further or gets there sooner - only what each one does when confronted
with the same situation. Outcome comparison is for physical testing.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Callable, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'beetlebot_risk_nav'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'tests'))

from beetlebot_risk_nav.baseline.baseline_controller import (  # noqa: E402
    BaselineAvoidanceController)
from beetlebot_risk_nav.core.footprint import FootprintChecker  # noqa: E402
from beetlebot_risk_nav.core.local_planner import LocalPlanner  # noqa: E402
from beetlebot_risk_nav.core.params import NavConfig  # noqa: E402
from synthetic import corridor, make_scan, room  # noqa: E402


class _Observation:
    """The small interface the baseline port expects."""

    def __init__(self, scan, stamp, pose, goal):
        self.scan = scan
        self.stamp = stamp
        self.pose = pose
        self.goal = goal
        self.velocity = (0.0, 0.0)
        self.true_pose = pose


class Situation:
    """One reproducible obstacle configuration, held static for a few cycles."""

    def __init__(self, name: str, description: str, goal=(3.0, 0.0), **scan_kwargs):
        self.name = name
        self.description = description
        self.goal = goal
        self.scan_kwargs = scan_kwargs

    def scan(self, stamp: float):
        return make_scan(stamp=stamp, **self.scan_kwargs)


SITUATIONS: List[Situation] = [
    Situation('clear corridor', 'nothing in the way',
              walls=corridor(1.5)),
    Situation('wall 1.2 m ahead', 'flat wall across the path',
              walls=[(1.2, -1.5, 1.2, 1.5)]),
    Situation('obstacle 0.9 m ahead', 'a box on the path, both sides open',
              circles=[(0.9, 0.0, 0.25)]),
    Situation('obstacle 0.6 m ahead', 'too close to drive past',
              circles=[(0.6, 0.0, 0.20)]),
    Situation('thin pole 0.8 m ahead', 'narrow object, easy to miss',
              circles=[(0.8, 0.0, 0.06)]),
    Situation('offset obstacle', 'box just off the centre line',
              circles=[(1.0, 0.22, 0.25)]),
    Situation('gap between obstacles', 'a passable gap straight ahead',
              circles=[(1.5, 0.9, 0.3), (1.5, -0.9, 0.3)]),
    Situation('corner', 'wall ahead and to the right',
              walls=[(1.2, -2.0, 1.2, 2.0), (0.0, -0.8, 1.2, -0.8)]),
    Situation('cluttered room', 'several objects at different ranges',
              walls=room(-4, -3, 4, 3),
              circles=[(1.4, 0.3, 0.25), (2.2, -0.9, 0.3), (2.8, 1.1, 0.2)]),
]


def clearance_of_command(planner: LocalPlanner, scan, v: float, w: float,
                         footprint: FootprintChecker) -> float:
    """Smallest footprint clearance along the rollout of a given command.

    This is the objective yardstick: both controllers are judged by the same
    geometry, using the same footprint, on the same measured points.
    """
    filtered = planner.filter.filter(scan)
    points = planner._static_points(filtered, planner._reach_radius())
    if not points:
        return float('inf')
    poses = planner.rollout(v, w, planner.rollout_times())
    worst = float('inf')
    for pose in poses:
        worst = min(worst, footprint.clearance_to_points(pose, points))
    return worst


def evaluate(situation: Situation, cycles: int = 5) -> Dict[str, object]:
    config = NavConfig()
    planner = LocalPlanner(config)
    baseline = BaselineAvoidanceController()
    baseline.reset()
    # Clearance is measured against the true chassis without the comfort margin,
    # so neither controller is credited or penalised for our inflation choice.
    yardstick = FootprintChecker(config.robot, inflation=0.0)

    new_command = (0.0, 0.0)
    base_command = (0.0, 0.0)
    for k in range(cycles):
        stamp = k * 0.1
        scan = situation.scan(stamp)
        result = planner.plan(scan, situation.goal, robot_velocity=planner.last_command)
        new_command = (result.command.v, result.command.w)
        base_command = baseline.control(_Observation(scan, stamp, (0.0, 0.0, 0.0),
                                                     situation.goal))

    scan = situation.scan(cycles * 0.1)
    return {
        'situation': situation,
        'new': new_command,
        'baseline': base_command,
        'new_clearance': clearance_of_command(planner, scan, new_command[0],
                                              new_command[1], yardstick),
        'baseline_clearance': clearance_of_command(planner, scan, base_command[0],
                                                   base_command[1], yardstick),
        'risk': result.assessment.level,
        'chosen': result.debug.chosen,
    }


def describe_command(command: Tuple[float, float]) -> str:
    v, w = command
    if abs(v) < 1e-6 and abs(w) < 1e-6:
        return 'stop'
    parts = []
    if v > 1e-6:
        parts.append('fwd {:.2f}'.format(v))
    elif v < -1e-6:
        parts.append('rev {:.2f}'.format(abs(v)))
    if w > 1e-6:
        parts.append('left {:.2f}'.format(w))
    elif w < -1e-6:
        parts.append('right {:.2f}'.format(abs(w)))
    return ' '.join(parts)


def _clr(value: float) -> str:
    if math.isinf(value):
        return '   n/a'
    return '{:6.3f}'.format(value)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--verbose', action='store_true',
                        help='also print what each situation represents')
    args = parser.parse_args(argv)

    print('Baseline (fixed thresholds) vs new navigation system')
    print('Identical synthetic scans; clearance is the worst footprint-to-obstacle')
    print('distance along each commanded motion, measured the same way for both.')
    print()
    header = '{:<24} {:<20} {:>7}  {:<20} {:>7}  {:<8}'.format(
        'situation', 'baseline', 'clr(m)', 'new system', 'clr(m)', 'risk')
    print(header)
    print('-' * len(header))

    safer = equal = worse = 0
    for situation in SITUATIONS:
        row = evaluate(situation)
        new_clear = row['new_clearance']
        base_clear = row['baseline_clearance']
        if not (math.isinf(new_clear) or math.isinf(base_clear)):
            if new_clear > base_clear + 0.01:
                safer += 1
            elif new_clear < base_clear - 0.01:
                worse += 1
            else:
                equal += 1
        print('{:<24} {:<20} {:>7}  {:<20} {:>7}  {:<8}'.format(
            situation.name,
            describe_command(row['baseline']), _clr(base_clear),
            describe_command(row['new']), _clr(new_clear),
            row['risk']))
        if args.verbose:
            print('    {}'.format(situation.description))

    print()
    print('Clearance of the commanded motion: new system safer in {}, comparable in {}, '
          'worse in {} of {} situations.'.format(safer, equal, worse, len(SITUATIONS)))
    print()
    print('This compares DECISIONS on identical inputs, not driving outcomes. It has')
    print('no robot model and no closed loop, so it cannot say which controller')
    print('reaches a goal sooner - only what each does in a given situation.')
    print('Outcome comparison requires physical testing on the BeetleBot.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
