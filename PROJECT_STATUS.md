# BeetleBot Autonomous Navigation - Project Status

**Objective:** given a start point and a goal point on the existing 2D map, drive the
physical VEEROBOT BeetleBot to the goal without colliding with static or dynamic
obstacles, stop within tolerance, and report SUCCESS.

| | |
|---|---|
| **Current phase** | Phase 1 complete |
| **Git checkpoint** | `phase-1-working` |
| **GitHub push status** | see "Git checkpoints" below |
| **Physical validation** | **NOT PERFORMED** - no ROS or robot in the development environment |

---

## Completed phases

### Phase 1 - Core navigation (COMPLETE)

LiDAR filtering, obstacle detection, basic tracking, four-level risk assessment,
adaptive velocity, footprint-aware collision checking and local obstacle
avoidance, integrated into a single `LocalPlanner.plan()` call and tested
end-to-end against synthetic scans.

## Verified robot facts

These are **not** guesses and must not be "tuned":

| Fact | Value | Source |
|---|---|---|
| `robot_radius` | 0.22 m | the robot's own `nav2_params.yaml` |
| Chassis | 0.375 m x 0.360 m | official VEEROBOT documentation |
| Platform | ROS 2 Jazzy, Raspberry Pi 5, RPLiDAR C1, STM32F405 | VEEROBOT documentation |
| Output topic | parameter, default `/cmd_vel_nav` | user decision; matches the working baseline |
| Navigation mode | node drives to the goal itself from `/goal_pose` | user decision (Nav2 is not producing commands) |

## Files / modules

```
baseline/lyra_control/obstacle_avoidance.py   PRESERVED original, byte-for-byte, never modified
src/beetlebot_risk_nav/beetlebot_risk_nav/
  core/geometry.py        2D primitives (angles, transforms, exact arc integration)
  core/types.py           shared data containers
  core/params.py          all configuration + validation
  core/scan_filter.py     LiDAR validity, safety-biased smoothing, speckle rejection
  core/clustering.py      obstacle detection; structure-vs-object classification
  core/tracker.py         basic tracking; windowed velocity; moving/static decision
  core/ttc.py             time-to-collision, braking distance
  core/risk.py            CLEAR / CAUTION / DANGER / CRITICAL assessment
  core/velocity.py        risk-adaptive speed envelope with braking guarantee
  core/footprint.py       exact rectangular or circular footprint collision checking
  core/local_planner.py   Phase 1 integration: 8 motion options, scored and checked
  baseline/baseline_controller.py   ROS-free port of the baseline, for comparison
tests/                    109 tests (unit + integration + edge/fault)
scripts/run_tests.sh      build check + import check + lint + tests
```

## Tests executed

Run with `./scripts/run_tests.sh`. Last run: **all checks passed**.

| Check | Result |
|---|---|
| Byte-compile (build) | pass - all modules compile |
| Import check | pass - 15 modules import without ROS |
| Lint (flake8) | pass - clean |
| Test suite | **109 passed**, 0 failed |
| Worst planner cycle time | < 30 ms asserted; ~3 ms measured on the dev machine |

Coverage by area: geometry 13, config 9, scan filtering 11, clustering 7,
footprint 9, TTC 10, tracking 9, risk 10, velocity 8, local planner integration 16.

## Defects found by testing and fixed

These were found by running the code, not by inspection:

1. **Bounding discs around walls swallowed free space.** A whole wall was covered by
   one 1.5 m disc that contained the robot's own position, so every motion looked
   like a collision and the robot never moved. Fixed by capping cluster discs at
   0.12 m so they hug surfaces.
2. **Phantom obstacle velocities up to 0.98 m/s on static walls.** Cluster boundaries
   slide along an extended surface as the robot moves, which is indistinguishable
   from motion for a single cluster. Fixed with windowed-displacement velocity plus
   excluding extended structure from the moving classification. Now measured at
   exactly 0.00 m/s phantom motion, while a 0.6 m/s crosser is still detected.
3. **A wall *behind* the robot throttled it to a crawl.** Proximity risk ignored
   direction. Fixed with bearing weighting and a forward-sector braking clearance.
4. **Clearance was ~13 cm pessimistic**, declaring CRITICAL with 40 cm of real space,
   because it subtracted a cluster radius and the robot's circumscribed radius
   instead of measuring the footprint. Fixed by using exact footprint distances.
5. **`stop` won on cost and fed back on itself.** A stopped robot has a low
   acceleration ceiling, which lowered the speed limit, which made stopping look
   even better. Fixed by making stop a fallback rather than a scored competitor.
6. **A coarse or decimated scan was silently discarded in full** (every point looked
   isolated), blinding the robot without any error. Fixed by scaling the speckle
   support radius with beam spacing.
7. **An empty scan was read as "no obstacles" and commanded full speed.** Fixed:
   absence of data is not evidence of clear space, so the robot holds still.
8. **Planner cost was 221 ms/cycle**, unusable at 10 Hz. Now ~3 ms.

## Known issues / limitations

- **No global planner.** The robot steers toward the goal and avoids locally, so a
  concave obstacle or a room whose exit faces away from the goal can trap it. This
  is a deliberate simplification. If physical testing shows trapping, the fix is an
  A* over the existing `/map`, which is already available.
- **ROS integration is not yet written** (Phase 2) and cannot be executed here: the
  development environment has no ROS, so all ROS interfaces remain unverified
  against a live system.
- A moving obstacle's current scan points stay in the static set for the length of
  the rollout, so the robot gives a slightly wider berth than strictly necessary.
  Deliberate: a mis-estimated velocity can then never open a hole in the check.

## Git checkpoints

| Tag | Meaning | Pushed |
|---|---|---|
| `phase-1-working` | Core navigation complete and tested | see commit log |

## Physical validation status

**Not performed.** No ROS, no robot, and no simulator in this environment. Every
result above is a software test result. Physical behaviour is unvalidated.
