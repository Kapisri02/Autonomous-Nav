#!/usr/bin/env python3
"""Summarise a navigation run log produced by the node.

    ./scripts/analyze_log.py beetlebot_logs/run_*.csv

This is the tool for the physical-testing feedback loop. It answers the
questions that matter after a run on the floor: did it reach the goal, how close
did it come to anything, where did the time go, did the safety layer fire, and
was the control loop keeping up. Every number comes from the log - nothing here
estimates or extrapolates.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import sys
from typing import Dict, List, Optional, Sequence

RISK_LEVELS = ('CLEAR', 'CAUTION', 'DANGER', 'CRITICAL')


def read_rows(path: str) -> List[Dict[str, str]]:
    with open(path, newline='') as handle:
        return list(csv.DictReader(handle))


def _float(row: Dict[str, str], key: str) -> Optional[float]:
    value = row.get(key, '')
    if value is None or value == '':
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return None if math.isnan(result) else result


def summarise(path: str) -> Dict[str, object]:
    rows = read_rows(path)
    if not rows:
        return {'path': path, 'error': 'log is empty'}

    times = [t for t in (_float(r, 'time') for r in rows) if t is not None]
    duration = (max(times) - min(times)) if times else 0.0

    clearances = [c for c in (_float(r, 'clearance') for r in rows) if c is not None]
    speeds = [abs(v) for v in (_float(r, 'cmd_v') for r in rows) if v is not None]
    cycles = [c for c in (_float(r, 'compute_ms') for r in rows) if c is not None]
    distances = [d for d in (_float(r, 'distance_to_goal') for r in rows) if d is not None]

    states = [r.get('state', '') for r in rows]
    final_state = states[-1] if states else ''
    risk_counts = {level: sum(1 for r in rows if r.get('risk_level') == level)
                   for level in RISK_LEVELS}

    estop_rows = [int(r.get('estop') or 0) for r in rows]
    estop_episodes = sum(1 for i, value in enumerate(estop_rows)
                         if value and (i == 0 or not estop_rows[i - 1]))

    # Travelled distance from the logged pose track.
    travelled = 0.0
    previous = None
    for row in rows:
        x, y = _float(row, 'x'), _float(row, 'y')
        if x is None or y is None:
            continue
        if previous is not None:
            travelled += math.hypot(x - previous[0], y - previous[1])
        previous = (x, y)

    stopped = sum(1 for v in speeds if v < 0.02)
    reversing = sum(1 for v in (_float(r, 'cmd_v') for r in rows)
                    if v is not None and v < -1e-6)

    return {
        'path': path,
        'cycles': len(rows),
        'duration': duration,
        'final_state': final_state,
        'succeeded': final_state == 'GOAL_REACHED',
        'travelled': travelled,
        'final_distance': distances[-1] if distances else None,
        'min_clearance': min(clearances) if clearances else None,
        'mean_clearance': sum(clearances) / len(clearances) if clearances else None,
        'mean_speed': sum(speeds) / len(speeds) if speeds else 0.0,
        'max_speed': max(speeds) if speeds else 0.0,
        'stopped_fraction': stopped / len(rows),
        'reverse_fraction': reversing / len(rows),
        'risk_counts': risk_counts,
        'recovery_cycles': sum(1 for s in states if s == 'RECOVERY'),
        'estop_episodes': estop_episodes,
        'mean_cycle_ms': sum(cycles) / len(cycles) if cycles else 0.0,
        'max_cycle_ms': max(cycles) if cycles else 0.0,
        'notes': rows[-1].get('notes', ''),
    }


def _fmt(value: Optional[float], spec: str = '{:.3f}') -> str:
    return 'n/a' if value is None else spec.format(value)


def print_report(summary: Dict[str, object], budget_ms: float = 100.0) -> None:
    if 'error' in summary:
        print('{}: {}'.format(summary['path'], summary['error']))
        return

    print('=' * 72)
    print('Run: {}'.format(os.path.basename(str(summary['path']))))
    print('=' * 72)
    outcome = 'SUCCESS' if summary['succeeded'] else 'NOT REACHED'
    print('  Outcome            : {} ({})'.format(outcome, summary['final_state']))
    print('  Reason             : {}'.format(summary['notes']))
    print('  Duration           : {:.1f} s over {} cycles'.format(
        summary['duration'], summary['cycles']))
    print('  Distance travelled : {:.2f} m'.format(summary['travelled']))
    print('  Final gap to goal  : {} m'.format(_fmt(summary['final_distance'], '{:.2f}')))
    print()
    print('  Safety')
    print('    Minimum clearance : {} m   <-- the headline safety number'.format(
        _fmt(summary['min_clearance'], '{:.3f}')))
    print('    Mean clearance    : {} m'.format(_fmt(summary['mean_clearance'], '{:.3f}')))
    print('    Emergency stops   : {}'.format(summary['estop_episodes']))
    print('    Recovery cycles   : {}'.format(summary['recovery_cycles']))
    print()
    print('  Motion')
    print('    Mean speed        : {:.3f} m/s (max {:.3f})'.format(
        summary['mean_speed'], summary['max_speed']))
    print('    Time stopped      : {:.1f} %'.format(100.0 * summary['stopped_fraction']))
    print('    Time reversing    : {:.1f} %'.format(100.0 * summary['reverse_fraction']))
    print()
    print('  Risk level distribution')
    counts = summary['risk_counts']
    total = max(1, sum(counts.values()))
    for level in RISK_LEVELS:
        share = 100.0 * counts[level] / total
        bar = '#' * int(share / 2.5)
        print('    {:<9} {:>5.1f} %  {}'.format(level, share, bar))
    print()
    print('  Control loop')
    print('    Mean cycle        : {:.1f} ms'.format(summary['mean_cycle_ms']))
    print('    Worst cycle       : {:.1f} ms'.format(summary['max_cycle_ms']))
    if summary['max_cycle_ms'] > budget_ms:
        print('    WARNING: worst cycle exceeded the {:.0f} ms control period. '
              'Raise lidar.decimation or lower control_frequency.'.format(budget_ms))
    print()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('logs', nargs='+', help='CSV log files (globs accepted)')
    parser.add_argument('--budget-ms', type=float, default=100.0,
                        help='control period in ms, for the overrun warning')
    parser.add_argument('--csv', action='store_true',
                        help='emit one summary row per run instead of a report')
    args = parser.parse_args(argv)

    paths: List[str] = []
    for pattern in args.logs:
        paths.extend(sorted(glob.glob(pattern)) or [pattern])

    summaries = []
    for path in paths:
        if not os.path.exists(path):
            print('no such log: {}'.format(path), file=sys.stderr)
            continue
        summaries.append(summarise(path))

    if not summaries:
        return 1

    if args.csv:
        writer = csv.writer(sys.stdout)
        writer.writerow(['run', 'succeeded', 'duration_s', 'travelled_m',
                         'min_clearance_m', 'mean_speed_mps', 'estops', 'max_cycle_ms'])
        for s in summaries:
            if 'error' in s:
                continue
            writer.writerow([os.path.basename(str(s['path'])), s['succeeded'],
                             round(s['duration'], 2), round(s['travelled'], 2),
                             _fmt(s['min_clearance']), round(s['mean_speed'], 3),
                             s['estop_episodes'], round(s['max_cycle_ms'], 1)])
        return 0

    for summary in summaries:
        print_report(summary, args.budget_ms)

    if len(summaries) > 1:
        reached = sum(1 for s in summaries if s.get('succeeded'))
        print('{} of {} runs reached the goal'.format(reached, len(summaries)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
