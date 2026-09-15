# BeetleBot Autonomous Navigation

Autonomous start-to-goal navigation for the VEEROBOT BeetleBot (Lyra).

Pick a destination in RViz, and the robot drives to it, avoiding static and
moving obstacles on the way, then stops and reports success.

```
   Start + Goal on the existing map
                 |
                 v
        Navigate toward goal  <-----------------+
                 |                              |
                 v                              |
          LiDAR filtering                       |
                 |                              |
                 v                              |
        Obstacle detection                      |
                 |                              |
                 v                              |
     Tracking: static or moving?                |
                 |                              |
                 v                              |
   Risk: CLEAR / CAUTION / DANGER / CRITICAL    |
                 |                              |
                 v                              |
        Adaptive speed envelope                 |
                 |                              |
                 v                              |
   Local avoidance: pick a safe manoeuvre       |
                 |                              |
                 v                              |
      Footprint collision check                 |
                 |                              |
                 v                              |
   Safety supervisor (can always override)      |
                 |                              |
                 v                              |
            /cmd_vel_nav ---> BeetleBot         |
                 |                              |
                 v                              |
        Goal reached? --- no ------------------+
                 |
                yes
                 v
             STOP -> SUCCESS
```

Priority is fixed: **safety > obstacle avoidance > goal progress**.

---

## Status

Software-complete and software-tested. **Not yet validated on the physical
robot** - see [PROJECT_STATUS.md](PROJECT_STATUS.md).

All results in this repository come from unit and integration tests against
deterministic synthetic inputs. There is no simulator here, and no claim is made
about physical behaviour.

---

## How it works

| Stage | What it does | Why it is there |
|---|---|---|
| **LiDAR filtering** | Drops NaN/out-of-range beams, suppresses speckle, smooths in the safety-biased direction only | Filtering must never be able to push an obstacle *further away* |
| **Obstacle detection** | Groups points into small discs; flags long connected runs as structure | A disc around a whole wall would swallow the free space beside it |
| **Tracking** | Associates clusters between scans, measures velocity over a time window | Tells a person walking across the path from a wall that only looks like it moves |
| **Risk** | Four levels from clearance and time-to-collision, weighted by bearing | Interpretable: every decision traces to one threshold in the log |
| **Adaptive velocity** | Risk scale, plus a braking cap `v <= sqrt(2a(d-margin))` | Guarantees the robot can stop inside the space it can actually see |
| **Local avoidance** | Scores 8 concrete manoeuvres, rejects any that collide | Small and legible; a blocked heading is expensive from far away |
| **Footprint check** | Exact rectangle (or Nav2's 0.22 m circle), not a point | A rectangular chassis sweeps its corners when it turns |
| **Safety supervisor** | Watchdogs, command validation, latched E-stop, hard limits | Independent of the planner, and can always override it |
| **Goal completion** | Stops within tolerance and latches SUCCESS | No further commands are issued once the goal is reached |

The entire decision core is pure Python with no ROS and no NumPy dependency, so
it can be tested anywhere and runs unchanged on the robot. Only
`nodes/beetlebot_nav_node.py` imports `rclpy`.

---

## The workspace

One workspace, two ROS packages:

```
~/beetlebot_ws/src/
├── map_selection_test/     map display + endpoint selection (publishes /map,
│                           reports the pose picked in RViz)
└── beetlebot_risk_nav/     autonomous navigation (drives to the goal, avoids
                            obstacles, publishes /cmd_vel_nav)
```

They share no code. `map_selection_test` logs the selected point; the navigation
node subscribes to `/goal_pose` itself and drives there. Either can run alone.

## Installation

```bash
mkdir -p ~/beetlebot_ws/src && cd ~/beetlebot_ws/src
git clone -b claude/nifty-fermat-hx9jyj \
    https://github.com/Kapisri02/Hopefully-final.git Hopefully-final
ln -s ~/beetlebot_ws/src/Hopefully-final/src/beetlebot_risk_nav   ~/beetlebot_ws/src/
ln -s ~/beetlebot_ws/src/Hopefully-final/src/map_selection_test   ~/beetlebot_ws/src/

# The map image is a binary asset and is not in git - copy it in once:
cp ~/ros2_ws/src/map_selection_test/maps/test.pgm \
   ~/beetlebot_ws/src/Hopefully-final/src/map_selection_test/maps/

cd ~/beetlebot_ws
rosdep install --from-paths src --ignore-src -r -y     # optional but recommended
colcon build --symlink-install
source install/setup.bash
```

Dependencies come from a standard ROS 2 Jazzy install (`rclpy`, `sensor_msgs`,
`geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`) plus `python3-yaml` and
`python3-pil`, which `map_selection_node` uses to read the map. The navigation
core itself needs nothing beyond the Python standard library.

---

## Running it

**Terminal 1 - robot hardware** (unchanged from your existing setup):

```bash
ssh veerobot@beetlebot-124.local
source /opt/ros/jazzy/setup.bash
source ~/lyra_ws/install/setup.bash
ros2 launch lyra_bringup robot.launch.py
```

**Terminal 2 - localisation against your existing map**, so that a goal picked
on the map means something. Whatever you normally use (AMCL / nav2 localisation)
is fine; all this package needs from it is the `map -> base_link` transform.

**Terminal 3 - map, endpoint selection and navigation, in one command:**

```bash
source ~/beetlebot_ws/install/setup.bash
ros2 launch beetlebot_risk_nav beetlebot_system.launch.py
```

This replaces the old four-terminal sequence. It publishes `/map`, listens for
the endpoint, and runs the navigation node. To use `nav2_map_server` as the map
publisher instead (it is lifecycle-managed; the launch configures and activates
it for you), add `use_map_server:=true`. Navigation itself never goes through
Nav2.

Navigation alone, without the map or endpoint-selection node:

```bash
ros2 launch beetlebot_risk_nav beetlebot_nav.launch.py
```

**Terminal 4 - RViz.** Set Fixed Frame to `map`, add a Map display on `/map`,
then use the **2D Goal Pose** tool to pick a destination. The selection node
logs the coordinates and the robot drives there and stops.

### If you are not running localisation

A goal picked on the map can only be driven to if something publishes
`map -> base_link`. Without it the node **refuses to move** and says so, rather
than comparing a map-frame goal against an odometry position and driving to the
wrong place. To work deliberately in the odometry frame:

```bash
ros2 launch beetlebot_risk_nav beetlebot_system.launch.py global_frame:=odom
```

Watch what it is thinking:

```bash
ros2 topic echo /beetlebot_nav/status
```

```
NAVIGATING 2.41 m to goal | risk CAUTION | clr 0.78 m | slight_left
```

Success is announced on the console and latched on `/beetlebot_nav/goal_reached`.

**Stopping:** Ctrl+C. The node publishes repeated zero-velocity commands on the
way out, so it never leaves the wheels turning - the same protection the
original script has.

### Without localisation

If `map -> base_link` is not being published, the node falls back to raw
odometry and says so in a warning. Navigation still works relative to where the
robot started, but a goal picked on the map will not mean what you expect. To
use odometry deliberately:

```bash
ros2 launch beetlebot_risk_nav beetlebot_nav.launch.py use_tf:=false global_frame:=odom
```

---

## Configuration

All tunables live in `config/beetlebot_nav.yaml`, generated from the dataclasses
in `core/params.py` so the two cannot drift apart. Override at launch:

```bash
ros2 launch beetlebot_risk_nav beetlebot_nav.launch.py cmd_vel_topic:=/cmd_vel
```

or at runtime:

```bash
ros2 param set /beetlebot_nav velocity.nominal_speed 0.15
```

The values worth knowing:

| Parameter | Default | Meaning |
|---|---|---|
| `goal.xy_tolerance` | 0.20 m | How close counts as "arrived" |
| `velocity.nominal_speed` | 0.20 m/s | Cruise speed with a clear path |
| `robot.footprint_length/width` | 0.375 / 0.360 m | **Verified** chassis size - not a tuning knob |
| `robot.robot_radius` | 0.22 m | **Verified**, from your `nav2_params.yaml` |
| `robot.footprint_inflation` | 0.06 m | Comfort margin around the chassis |
| `risk.caution/danger/critical_distance` | 1.20 / 0.60 / 0.25 m | Where the robot slows, gets cautious, stops |
| `safety.estop_distance` | 0.18 m | Emergency stop trigger |
| `lidar.decimation` | 1 | Raise to 2 if the control loop cannot keep up |

The configuration is validated at startup; the node refuses to start on a
contradictory configuration rather than driving with it.

---

## Testing

```bash
./scripts/run_tests.sh
```

Runs a byte-compile build check, an import check, flake8, and the full test
suite (unit, integration, edge/fault, and the ROS node against stubbed ROS
modules). Everything is deterministic - no random seeds, no timing dependence.

Individual areas:

```bash
PYTHONPATH=src/beetlebot_risk_nav:tests pytest tests/test_local_planner.py -v
PYTHONPATH=src/beetlebot_risk_nav:tests pytest tests/test_safety.py -v
```

Compare against the preserved baseline on identical inputs:

```bash
./scripts/compare_baseline.py --verbose
```

---

## After a physical run

Every navigation attempt writes a CSV to `beetlebot_logs/`. Summarise it:

```bash
./scripts/analyze_log.py beetlebot_logs/run_*.csv
```

You get the outcome, the minimum clearance reached, where the time went, how
often the safety layer fired, and whether the control loop kept up. That is the
evidence to send back when something needs fixing - it says what the robot saw
and why it decided what it did.

---

## Troubleshooting

**The robot does not move at all.**
Check `/beetlebot_nav/status`. Common causes:
- `waiting for a goal` - no goal has been set; use RViz 2D Goal Pose.
- `waiting for a robot pose` - no TF and no `/odom`. Check your localisation.
- `insufficient LiDAR data` - the scan is empty or mostly NaN. Check
  `ros2 topic hz /scan`. The robot deliberately holds still rather than driving
  on data it does not have.
- `SENSOR_FAULT` - scans or odometry stopped arriving. Check the bringup terminal.

**It stops well short of obstacles.**
That is the risk envelope doing its job. If it is too cautious for your space,
raise `risk.caution_distance` and `risk.danger_distance`. Do not reduce
`robot.footprint_*` - those describe the physical robot.

**It stops and will not restart.**
Look for `EMERGENCY_STOP` in the status. It releases once there is
`safety.estop_clear_distance` (0.30 m) of room and the minimum latch time has
passed. If the robot is boxed in, it will reverse slowly out, but only when the
space behind it is verified clear.

**It wanders or oscillates near obstacles.**
Increase `planner.weight_smoothness`, or reduce `planner.slight_turn`. Check the
log first: `analyze_log.py` shows whether it was flipping between manoeuvres.

**The robot will not move and the status says "refusing to navigate".**
The goal is in the `map` frame but no `map -> base_link` transform exists, so
only the odometry position is known. Start your localisation, or relaunch with
`global_frame:=odom` to navigate in the odometry frame deliberately.

**The map does not appear in RViz.**
Check Fixed Frame is `map` and the Map display topic is `/map`. If you passed
`use_map_server:=true`, the lifecycle activation happens ~2 s after launch -
check for `Transitioning successful`. Do not run `map_selection_node` and
`nav2_map_server` as map publishers at the same time: two transient-local
publishers on `/map` leave it ambiguous which one a subscriber latches.

**`Failed to load map image`.**
`test.pgm` has not been copied into `src/map_selection_test/maps/`. It is a
binary asset and is not stored in git - see `maps/README.md`.

**It never reaches a goal on the far side of a wall.**
Expected: there is no global planner, by design. The robot steers toward the
goal and avoids locally, so a route that needs to go *away* from the goal first
will not be found. Give it intermediate goals, or see PROJECT_STATUS.md for the
planned fix.

**The control loop is too slow** (warning about the cycle budget).
Set `lidar.decimation: 2`, or reduce `control_frequency` to 5.0. Measured
worst-case on the development machine is ~13 ms against a 100 ms budget, so
there is considerable headroom, but the Pi 5 is slower.

**Commands are published but the robot does not respond.**
Check which topic your base actually consumes:
`ros2 topic info /cmd_vel_nav`. If your mux expects something else, set
`cmd_vel_topic` accordingly.

---

## The baseline

The original obstacle-avoidance script is preserved byte-for-byte at
`baseline/lyra_control/obstacle_avoidance.py` and is never modified. It remains
a working fallback, and `scripts/compare_baseline.py` uses a faithful ROS-free
port of its logic for side-by-side comparison.

---

## Repository layout

```
baseline/lyra_control/obstacle_avoidance.py   preserved original
src/beetlebot_risk_nav/                       ROS package: autonomous navigation
  beetlebot_risk_nav/core/      pure-Python decision core (no ROS, no NumPy)
  beetlebot_risk_nav/nodes/     ROS 2 adapter
  beetlebot_risk_nav/baseline/  ROS-free port of the baseline
  config/, launch/              parameters, beetlebot_nav + beetlebot_system launch
src/map_selection_test/                       ROS package: map + endpoint selection
  map_selection_test/map_selection_node.py    preserved verbatim
  maps/test.yaml                installed to share/; test.pgm copied in separately
  launch/map_selection.launch.py
tests/                          the test suite
scripts/run_tests.sh            build + lint + tests
scripts/analyze_log.py          summarise a run
scripts/compare_baseline.py     baseline vs new system
docs/ROS_INTERFACES.md          topics, frames, and assumptions
```
