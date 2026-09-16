# BeetleBot Autonomous Navigation - Project Status

**Objective:** given a start point and a goal point on the existing 2D map, drive the
physical VEEROBOT BeetleBot to the goal without colliding with static or dynamic
obstacles, stop within tolerance, and report SUCCESS.

| | |
|---|---|
| **Current phase** | Phase 4 - endpoint selection merged into one workspace |
| **Git checkpoint** | `final-software-ready` (Phase 3); Phase 4 on the branch |
| **GitHub push status** | branch pushed; **tag pushes blocked by policy (HTTP 403)** - see below |
| **Physical validation** | **NOT PERFORMED** (robot untouched) |
| **Real ROS 2 validation** | **PARTIAL - both nodes launch and their interfaces are confirmed on ROS 2 Jazzy** |

---

## Completed phases

### Phase 1 - Core navigation (COMPLETE)
LiDAR filtering, obstacle detection, basic tracking, four-level risk assessment,
adaptive velocity, footprint-aware collision checking and local obstacle
avoidance, integrated into one `LocalPlanner.plan()` call.

### Phase 2 - Safety and robot integration (COMPLETE)
Independent safety supervisor (sensor watchdogs, command validation, latched
emergency stop, hard speed and rate limits), the full start-to-goal flow with
goal-tolerance completion and a latched SUCCESS state, baseline-style
reverse-then-turn recovery, and the ROS 2 node, package, launch file and
parameter file.

### Phase 3 - Final software completion (COMPLETE)
Per-cycle CSV logging, run analysis (`scripts/analyze_log.py`), baseline
comparison (`scripts/compare_baseline.py`), README with installation, run,
configuration, testing and troubleshooting instructions, and
`docs/ROS_INTERFACES.md`.

### Phase 4 - Workspace merge (COMPLETE, unbuilt)

`map_selection_test` (map display + endpoint selection) and `beetlebot_risk_nav`
(autonomous navigation) are now two packages in one workspace, buildable with a
single `colcon build --symlink-install`. A unified
`beetlebot_system.launch.py` replaces the four-terminal startup.

**What the endpoint-selection node actually does** (read from its source, not
assumed): it publishes `/map` itself from `test.yaml`/`test.pgm` with
transient-local durability, and its `/goal_pose` callback **logs** the selected
X/Y/Yaw and publishes nothing. There is no start/goal state machine.

So **`/goal_pose` is goal-only**, and the start comes from localisation - which
matches how the navigation node already worked. Both nodes subscribe to
`/goal_pose` independently; neither relays to the other, and neither can block
the other. No change was needed to the navigation algorithm.

Five issues found while merging:

1. **No localisation in the documented workflow.** The four-terminal sequence
   starts no AMCL, so `map -> base_link` does not exist. The navigation node
   accepted a map-frame goal, fell back to the odometry position, warned once and
   **drove anyway** - a map-frame goal compared against an odom-frame position,
   which sends the robot confidently to the wrong place. It now refuses to move
   in that combination and says why. `global_frame:=odom` navigates in the
   odometry frame deliberately.
2. **`nav2_map_server` and `map_selection_node` both publish `/map`.** Two
   transient-local publishers on one topic leave it ambiguous which a subscriber
   latches. The unified launch uses one, selectable with `use_map_server`.
3. **`maps/` was never installed.** `setup.py` did not list it, which is why the
   run command needed an absolute `/home/kapisri02/ros2_ws/...` path. Maps now
   install into `share/map_selection_test/maps/` and are found via
   `get_package_share_directory()`.
4. **Undeclared runtime dependencies.** The node imports `yaml` and `PIL`;
   `package.xml` declared neither, so a clean machine would build the package and
   then fail inside `publish_map()`. Both are now declared.
5. **`package.xml` metadata was left as `TODO`** for description and license.

The node's own source is preserved **verbatim** (asserted by a test). Only
packaging metadata around it changed.

---

## Verified robot facts

Not guesses, and not tuning knobs:

| Fact | Value | Source |
|---|---|---|
| `robot_radius` | 0.22 m | the robot's own `nav2_params.yaml` |
| Chassis | 0.375 m x 0.360 m | official VEEROBOT documentation |
| Platform | ROS 2 Jazzy, Raspberry Pi 5, RPLiDAR C1, STM32F405 | VEEROBOT documentation |
| Output topic | parameter, default `/cmd_vel_nav` | user decision; matches the working baseline |
| Navigation mode | node drives to the goal itself from `/goal_pose` | user decision (Nav2 is not producing commands) |

---

## Files / modules

```
baseline/lyra_control/obstacle_avoidance.py   PRESERVED original, byte-for-byte
src/beetlebot_risk_nav/
  beetlebot_risk_nav/core/
    geometry.py        2D primitives, exact arc integration
    types.py           shared data containers
    params.py          all configuration + startup validation
    scan_filter.py     validity, safety-biased smoothing, speckle rejection
    clustering.py      obstacle detection, structure-vs-object classification
    tracker.py         tracking, windowed velocity, moving/static decision
    ttc.py             time-to-collision, braking distance
    risk.py            CLEAR / CAUTION / DANGER / CRITICAL
    velocity.py        adaptive speed envelope with braking guarantee
    footprint.py       exact rectangular or circular collision checking
    local_planner.py   8 motion options, scored and footprint-checked
    safety.py          watchdogs, E-stop, command limits
    navigator.py       start-to-goal flow, recovery, SUCCESS state
    logging_utils.py   per-cycle CSV logging
  beetlebot_risk_nav/nodes/beetlebot_nav_node.py   ROS 2 adapter (only rclpy user)
  beetlebot_risk_nav/baseline/baseline_controller.py  ROS-free baseline port
  config/beetlebot_nav.yaml   generated from NavConfig so they cannot drift
  launch/beetlebot_nav.launch.py
  package.xml, setup.py       ament_python package
tests/                        170 tests
scripts/run_tests.sh          build + import + lint + tests
scripts/analyze_log.py        summarise a run
scripts/compare_baseline.py   baseline vs new system
README.md, docs/ROS_INTERFACES.md
```

---

## Tests executed

`./scripts/run_tests.sh` - last run: **ALL SOFTWARE CHECKS PASSED**.

| Check | Result |
|---|---|
| Byte-compile (build) | pass |
| Import check | pass - 17 modules import without ROS |
| Lint (flake8) | pass - clean |
| Test suite | **186 passed**, 0 failed |

By area: geometry 13, config 9, scan filtering 12, clustering 7, footprint 9,
TTC 10, tracking 9, risk 10, velocity 8, local planner 17, safety 20,
navigator 14, ROS node 19, logging/tools 9, system integration 4,
workspace layout 11.

### Phase 3 verification checklist

| Requirement | Status | Evidence |
|---|---|---|
| Package builds | **pass (partial)** | byte-compile + import of every module. `colcon` is not available here, so the ament build itself is unverified. |
| Nodes launch | **not verifiable here** | No ROS. The node is constructed and driven against stubbed ROS modules. |
| ROS interfaces correct | **structurally verified** | Topics, types and conversions asserted in `test_ros_node.py`; not checked against live ROS. |
| Pipeline integrated | pass | `test_system_integration.py` |
| Risk assessment works | pass | `test_risk.py`, 10 tests |
| Adaptive velocity works | pass | `test_velocity.py`, including the braking guarantee |
| Local motion selection works | pass | `test_local_planner.py`, 17 tests |
| Footprint checking works | pass | exact vs polygon reference over 5,000 random poses |
| Safety supervision works | pass | `test_safety.py`, 20 tests |
| Failure handling works | pass | empty/NaN scans, stale sensors, NaN commands, timeouts, recovery |
| Baseline available | pass | original preserved; `test_logging_and_tools.py` asserts it is unmodified |
| Evaluation / logging works | pass | log written, read back and summarised in tests |

### Measured performance

Worst planner cycle **~13 ms** against a 100 ms budget at 10 Hz, on the
development machine. The Pi 5 is slower; `lidar.decimation` is the lever if
needed, and the node warns when it overruns.

### Baseline comparison (decisions on identical inputs)

`./scripts/compare_baseline.py` over 9 situations: the new system commands a
motion with more clearance in 7, comparable in 1, less in 1 (it stops where the
baseline reverses). In three situations - a thin pole, an offset box, a box
0.9 m ahead - the baseline commands a motion with **negative** clearance, i.e.
into the obstacle, because its fixed +/-15 degree front window does not see them.
This compares decisions, not driving outcomes.

---

## Defects found by testing and fixed

Found by running the code, not by inspection:

1. **Bounding discs around walls swallowed free space** - a wall was covered by one
   1.5 m disc containing the robot's own position, so every motion looked like a
   collision. Fixed by capping cluster discs at 0.12 m.
2. **Phantom obstacle velocities up to 0.98 m/s on static walls** - sliding cluster
   boundaries are indistinguishable from motion for one cluster. Fixed with
   windowed-displacement velocity plus excluding extended structure. Now measured
   at 0.00 m/s phantom motion while a 0.6 m/s crosser is still detected.
3. **A wall behind the robot throttled it to a crawl** - proximity risk ignored
   direction. Fixed with bearing weighting and forward-sector braking clearance.
4. **Clearance was ~13 cm pessimistic**, declaring CRITICAL with 40 cm of real
   space. Fixed by measuring exact footprint distances.
5. **`stop` won on cost and fed back on itself** - a stopped robot has a low
   acceleration ceiling, which lowered the speed limit, which made stopping look
   better still. Fixed by making stop a fallback, not a scored competitor.
6. **A coarse or decimated scan was silently discarded in full**, blinding the robot
   with no error. Fixed by scaling the speckle support radius with beam spacing.
7. **An empty scan read as "no obstacles" and commanded full speed.** Fixed:
   absence of data is not evidence of clear space.
8. **Planner cost 221 ms/cycle**, unusable at 10 Hz. Now ~3 ms typical, 13 ms worst.
9. **The emergency stop could not be escaped** - it zeroed the recovery's reverse
   too, so anything set down within 18 cm froze the robot permanently. A slow
   retreat is now allowed, with forward motion and rotation still blocked, and
   only when the space behind has been checked.
10. **The reason for a failure was lost** after the cycle it happened on, leaving a
    bare "navigation failed".
11. **A goal in an unresolvable frame was silently accepted**, which would have
    driven confidently to the wrong place. Now refused with an error.

---

## Known issues / limitations

- **No global planner.** The robot steers toward the goal and avoids locally, so a
  route that must initially lead *away* from the goal (a concave obstacle, a room
  whose exit faces the wrong way) will not be found. Deliberate simplification.
  If physical testing shows trapping, the fix is A* over the existing `/map`.
- **Nothing ROS has been executed.** No ROS in this environment: `colcon build`,
  node launch, TF, QoS and message transport are unverified. The node is tested
  against stubs, which exercises our code but not ROS.
- **The LiDAR is assumed to be at the centre of rotation.** If it is mounted
  forward or aft, set `robot.footprint_offset_x`. This is the assumption most
  likely to matter physically - see `docs/ROS_INTERFACES.md`.
- A moving obstacle's current scan points stay in the static set for the length of
  the rollout, so the robot gives a slightly wider berth than strictly necessary.
  Deliberate: a mis-estimated velocity then cannot open a hole in the check.

---

## Git checkpoints

| Tag | Meaning |
|---|---|
| `phase-1-working` | Core navigation complete and tested |
| `phase-2-working` | Safety + ROS integration complete |
| `final-software-ready` | Software-complete, 170 tests passing |

All three exist as local tags and as commits on the pushed branch
`claude/nifty-fermat-hx9jyj`.

**Tag pushes are rejected by this environment with HTTP 403** (organisation
egress policy; the proxy documentation states policy denials must be reported
rather than retried). Branch pushes succeed. To publish the tags from a machine
with normal GitHub access:

```bash
git fetch origin
git push origin phase-1-working phase-2-working final-software-ready
```

---

## Verified on real ROS 2 Jazzy (laptop, no robot)

First execution of this project under real ROS, from `~/beetlebot_ws`:

| Item | Result |
|---|---|
| `colcon build --symlink-install` | both packages built |
| Package discovery | `/beetlebot_nav` and `/map_selection_node` both running from one workspace |
| Map from the packaged share path | `Published map: 480 x 480 cells, 0.05 m/cell, origin [-5, -12.0, 0.0]` - no `~/ros2_ws` path involved |
| Subscriptions | `/scan` LaserScan, `/odom` Odometry, `/goal_pose` PoseStamped, `/tf`, `/tf_static` |
| Publications | `/cmd_vel_nav` Twist, `/beetlebot_nav/status` String, `/beetlebot_nav/goal_reached` Bool |
| Parameters | the full dotted set is exposed and settable |
| Configuration banner | footprint 0.375 x 0.360 m rectangular, inflation 0.060 m, 10 Hz |
| Behaviour with no sensors | `waiting for a robot pose (TF or odometry)`, zero velocity - correct refusal |
| Endpoint-selection workflow | confirmed working by the user: RViz shows `/map`, a 2D Goal Pose click reaches both nodes from one workspace (reported, terminal output not captured here) |

Still unverified on real ROS: behaviour with live `/scan` and `/odom`, TF from a
running localisation, control-loop timing on the Raspberry Pi 5, and anything
involving the robot itself.

The `BrokenPipeError` from `ros2 param list ... | head -30` is `ros2cli` reacting
to the closed pipe, not a fault in this package.

### Defects this first real-ROS run exposed

1. **A blocking TF lookup every cycle in odometry mode.** With
   `global_frame:=odom` the node still asked TF for `odom -> base_link`.
   `lookup_transform` blocks for its whole timeout when the transform is
   unavailable, so it spent 50 ms of every 100 ms control period waiting for
   something odometry already provides. TF is now skipped when the working frame
   is the odometry frame.
2. **The cycle-budget watchdog was blind to it.** It timed `plan()` only, so
   anything slow outside the planner could consume the control period silently.
   It now times the whole cycle - in a `finally`, so the early-exit paths (no
   pose yet, goal refused) are covered too, which is precisely where a slow TF
   lookup shows up.
3. **A misleading warning.** In odometry mode it claimed the goal would not be
   driven to "until localisation is running", which is false - no localisation is
   needed when goal and pose are both in the odometry frame.

---

## Old workspace: what is safe to remove

**Nothing yet.** Remove nothing until `~/beetlebot_ws` has actually built and the
endpoint-selection workflow has been verified running from it - that has not
happened, because no ROS environment was available here.

Once `colcon build --symlink-install` succeeds in `~/beetlebot_ws`, `/map`
appears in RViz and `map_selection_node` logs a pose picked with 2D Goal Pose,
these become safe:

| Path | Safe to remove after verification | Why |
|---|---|---|
| `~/ros2_ws/build/`, `~/ros2_ws/install/`, `~/ros2_ws/log/` | yes | Regenerated by any build; contain nothing original. |
| `~/ros2_ws/src/map_selection_test/` | yes, **after** `test.pgm` is copied across | Source now lives in the repository. |
| `~/ros2_ws/` itself | yes, if it holds nothing else | Check with `ls ~/ros2_ws/src` first - remove only if `map_selection_test` is the sole package. |

Keep a copy of `test.pgm` somewhere outside both workspaces until the new one is
verified: it is the one artefact that cannot be regenerated from the repository.

## Physical validation status

**Not performed.** No ROS, no robot, no simulator in this environment. Every
result above is a software test result against deterministic synthetic inputs.

Suggested first physical checks, in order:
1. `colcon build --packages-select beetlebot_risk_nav` actually builds.
2. The node launches and prints its configuration banner.
3. With the robot on blocks, confirm `/cmd_vel_nav` carries sensible commands.
4. Confirm the LiDAR mounting offset, and set `robot.footprint_offset_x` if needed.
5. Short goal (1-2 m) in open space.
6. Goal with a static obstacle on the path.
7. Goal with a person walking across the path.
8. Ctrl+C mid-run: the wheels must stop immediately.

Then send back `beetlebot_logs/run_*.csv` and `./scripts/analyze_log.py` output.
