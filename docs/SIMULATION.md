# BeetleBot simulation, and what it established about the navigation code

This document records a verification exercise: a Gazebo Harmonic simulation of
the VEEROBOT BeetleBot was built to the dimensions VEEROBOT publishes, and the
**existing** `beetlebot_risk_nav` package was run against it unmodified. It
reports what was demonstrated, what failed, and what remains unverified.

Nothing here is a claim about the physical robot. It is a claim about the code
running against a model of the robot.

---

## 1. Where the robot's dimensions come from

Every number in `beetlebot_sim/urdf/beetlebot.urdf.xacro` is traceable to
VEEROBOT's own documentation, mirrored verbatim at
[SMARTS-LAB/Documention](https://github.com/SMARTS-LAB/Documention) (commit
`c9eb011`) from <https://docs.veerobot.com/ros-robots/beetle-bot>. The source
page for each value is named in the xacro next to the value itself.

| Quantity | Value | Where it is stated |
|---|---|---|
| Dimensions L x W x H | 375 x 360 x 245 mm | Introduction, "Technical Highlights" |
| Weight | ~2.2 kg with battery | Introduction |
| **Drive type** | **4-Wheel Skid-Steer** | Introduction |
| Max speed | 1.0 m/s | Introduction; Hardware Familiarization |
| Velocity clamp in firmware | 1.0 m/s linear, 2.0 rad/s angular | Hardware Familiarization |
| Acceleration ramp | 5 RPM per 50 ms cycle (`MAX_RPM_STEP_PER_CYCLE`) | Introduction |
| `base_footprint` -> `base_link` | 163 mm | Robot Simulation with Gazebo |
| Wheelbase (front-back) | 181 mm | Robot Simulation with Gazebo |
| Track width (left-right) | 290 mm | Robot Simulation with Gazebo |
| Wheel diameter | 130 mm (radius 0.065) | Hardware Familiarization; Gazebo page |
| Wheel width | ~40 mm | Hardware Familiarization |
| `fl_link` origin | (0.094, 0.145, -0.098) | Robot Simulation with Gazebo |
| `bl_link` origin | (-0.087, 0.145, -0.098) | Robot Simulation with Gazebo |
| **LiDAR position** | **85 mm forward, 206 mm above ground** | Hardware Familiarization; Sensor Data Visualization |
| LiDAR | RPLidar C1, 360 deg, 12 m, 10 Hz, ~0.5 deg | Hardware Familiarization |
| Camera | Pi Camera V1.3 (OV5647), 88 mm high, ~54 deg HFOV | Hardware Familiarization |
| IMU | LSM6DSRTR, I2C, 100 Hz, +/-2g | Hardware Familiarization |
| IMU noise | 0.04-0.05 m/s^2 accel, 0.003-0.004 rad/s gyro | IMU Signal Processing |
| Encoders | 900 CPR x4 quadrature = 3600 ticks/wheel rev | Hardware Familiarization |
| Motors | JGB37-3530, 12 V, 37:1, 150 RPM max | Hardware Familiarization |
| Odometry accuracy | +/-3-5% linear, +/-5-10% angular | Introduction |
| Turning | in-place rotation, zero radius | Introduction |

These are internally consistent, which is the strongest evidence available that
they describe one real robot rather than three different ones:

```
wheel centre height = 0.163 - 0.098 = 0.065 m = wheel radius          agrees
wheelbase           = 0.094 - (-0.087) = 0.181 m = 18.1 cm            agrees
track width         = 2 x 0.145 = 0.290 m = 29 cm                     agrees
max speed           = 150 rpm / 60 x pi x 0.130 = 1.02 m/s ~ 1.0 m/s  agrees
```

**The repository's own "verified" values check out.** `footprint_length/width`
= 0.375 / 0.360 m matches the published chassis exactly, and
`lidar.mount_offset_x` = 0.085 with a 0.206 m height matches two separate
vendor pages. The offset was previously listed in `PROJECT_STATUS.md` as "the
assumption most likely to matter physically"; it is no longer an assumption.

### Not established

* **Chassis vertical shape.** The footprint and overall height are published;
  how the body is distributed between them is not. The model uses a 60 mm slab
  that clears the wheel tops and stays below the LiDAR. This does not affect
  2-D navigation.
* **Camera x, IMU position.** Only the camera's height is published. Neither
  link is used by the navigation code; they exist so the TF tree matches.
* **Per-part masses.** Only the 2.2 kg total is published.
* **LiDAR noise.** Not published, so the simulated scan is noise-free. The real
  scan is not. This makes the simulation optimistic.
* **VEEROBOT ships no simulation.** Their Gazebo page is marked
  "TODO: Feature Not Yet Implemented" - `beetlebot_description` does not exist
  yet. There was no vendor URDF to reuse, and no third-party BeetleBot
  simulation was found. The model here was built from the documented numbers.

---

## 2. What the simulation provides

`beetlebot_sim` stands in for `lyra_bringup robot.launch.py`. It offers the
interfaces the navigation package already expected - the package was not
changed to suit it.

| Interface | Type | Verified behaviour |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 720 beams over 360 deg, 9.86 Hz, 0.1-12 m, frame `lidar_link` |
| `/odom` | `nav_msgs/Odometry` | frame `odom`, child `base_footprint`, 30 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` | 98.6 Hz (documented rate 100 Hz) |
| `/joint_states` | `sensor_msgs/JointState` | four wheel joints |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | consumed by the simulated skid-steer drive |
| TF | - | `odom` -> `base_footprint` -> `base_link` -> wheels/sensors |

The LiDAR geometry was checked against the world rather than assumed. With a
0.15 m radius pole at x = 2.0 m, the forward beam read **1.766 m** against a
prediction of 2.0 - 0.085 - 0.15 = **1.765 m**. The sensor is where the
documentation says it is, and `/scan` is consistent with it to a millimetre.

Run it headless (no GPU needed - Gazebo renders the LiDAR under llvmpipe):

```bash
xvfb-run -a ros2 launch beetlebot_sim beetlebot_sim_nav.launch.py \
    world:=beetlebot_obstacles global_frame:=odom
```

Worlds: `beetlebot_empty`, `beetlebot_obstacles`, `beetlebot_corridor`,
`beetlebot_clutter`, `beetlebot_confined`.

---

## 3. Scenario results

The navigation node, its own launch file and its own parameter file. Goal
tolerance 0.20 m throughout. "default" is the shipped configuration.

| # | Scenario | World | Configuration | Result |
|---|---|---|---|---|
| 1 | Point-to-point, 3 m | empty | default | **reached**, 0.199 m, 16 s |
| 2 | Past 4 static obstacles, 9 m | obstacles | default | **stalled** at the first pole |
| 2A | same | obstacles | `velocity.scale_danger` 0.3 -> 0.6 | **reached**, 12.94 m path |
| 2B | same | obstacles | `risk.danger_distance` 0.6 -> 0.35 | **stalled** (no help) |
| 2C | same | obstacles | `planner.sim_time` 3.0 -> 6.0 | **reached**, 16.61 m path |
| 3 | Corridor + 1.2 m doorway, 8 m | corridor | default | **reached**, 0.197 m, 8.21 m path |
| 4 | Cluttered room, 8 m | clutter | default | **stalled** after 3.87 m |
| 4B | same | clutter | `planner.sim_time` 6.0 | **reached**, 13.95 m path |
| 5 | 180 deg turn in a dead end | confined | default | **stalled** after 0.68 m |
| 5B | same | confined | `planner.sim_time` 6.0 | **stalled** (different cause) |
| 6 | Corridor in the **map** frame, AMCL | corridor | default | **reached**, 0.174 m |
| 7 | Three goals in sequence | corridor | `sim_time` 6.0 | **2 of 3** reached |
| 8A | Offset start (1.5, 0.6, 0.6 rad), map frame, AMCL | corridor | `sim_time` 6.0 | **reached**, 0.200 m |
| 8B | Offset start (0.5, -0.7, -0.5 rad), map frame, AMCL | corridor | `sim_time` 6.0 | **stalled** at the doorway |

Also observed across these runs: the emergency stop never fired spuriously (0
E-stops in every run); the recovery behaviour engaged on every stall and
released again; the control loop ran at **1.0 ms mean, 1.9 ms worst** against a
100 ms budget; and shutdown always published zero velocity.

### Localisation and mapping

* **AMCL localisation works** (scenarios 6, 8A, 8B). With `global_frame:=map`
  the node correctly **refused to move** until AMCL published
  `map -> base_link`, logged why, and then drove once the transform appeared -
  exactly the designed behaviour.
* Navigating from an offset start pose in the map frame reached a fixed world
  goal (8A), which is the real test that localisation is being used and not
  merely tolerated.
* **SLAM was not completed.** `slam_toolbox` starts and activates against the
  simulation but did not publish `/map` within the time available, so the map
  used for AMCL was generated from the world's own geometry
  (`beetlebot_sim/maps/corridor.*`) rather than scanned. The repository under
  test does not use SLAM - it consumes `map -> base_link` from whatever
  localisation is running - so this does not affect the verdict on the code.

---

## 4. The one defect that matters

**Three of five scenario types stall in the shipped configuration**, always
with the same signature: risk `DANGER`, speed capped near 0.06 m/s, the planner
falling back to `stop`, then `no progress for 45 s` and `FAILED`.

The mechanism is arithmetic, and it is in `local_planner.motion_options()`:
every manoeuvre's linear velocity is scaled by the current risk-limited speed.

```
DANGER  ->  speed_scale 0.3  ->  0.2 x 0.3 = 0.06 m/s
rollout  =  0.06 m/s x planner.sim_time 3.0 s  =  0.18 m
```

The planner therefore decides whether a manoeuvre clears an obstacle by
simulating **0.18 m of travel - less than the robot's own inflated front
overhang (0.2475 m)**. No turn can demonstrate a gain over that distance, so
every option scores alike, `stop` wins as the fallback, and the robot waits in
front of an obstacle it has ample room to drive around.

Two independent changes fix it, and both act on the same product `v x sim_time`:

| Change | Rollout | Scenario 2 | Scenario 4 |
|---|---|---|---|
| none (shipped) | 0.18 m | stalled | stalled |
| `velocity.scale_danger` 0.3 -> 0.6 | 0.36 m | reached | - |
| `planner.sim_time` 3.0 -> 6.0 | 0.36 m | reached | reached |

Lowering `risk.danger_distance` does **not** help (scenario 2B), which confirms
the binding constraint is the rollout distance and not the risk threshold.

**Classification: B, a configuration problem** - the algorithm is sound and
becomes effective with a parameter change; no code change was required to make
it navigate. It is recorded here rather than silently patched, because both
remedies raise the speed the robot carries toward obstacles, and that is a
decision about physical safety margins that belongs to whoever runs the robot.

Scenario 5 (a 180 degree turn in a dead end) stalls for a **different** reason
and is not fixed by more lookahead: in a 1.35 m pocket the side walls sit inside
`risk.danger_distance` permanently, so the robot never leaves `DANGER` whatever
it does. Scenario 8B stalls at a doorway when approaching at an angle. Both are
consistent with the documented "no global planner" limitation.

---

## 5. Other findings

| # | Finding | Class |
|---|---|---|
| 1 | `config/beetlebot_nav.yaml` had **drifted from the dataclasses** despite the file header and README both claiming it is generated from them and "cannot drift apart". Four keys were missing - `lidar.mount_offset_x`, `lidar.mount_offset_y`, `cluster.truncation_margin`, `risk.closing_speed_threshold` - and `lidar.field_of_view` was truncated. **Fixed**, and a test now enforces the invariant. The LiDAR mounting offset being the parameter you could not find in the parameter file is the part that mattered. | A |
| 2 | `docs/ROS_INTERFACES.md` and `PROJECT_STATUS.md` told the reader to set `robot.footprint_offset_x` for a LiDAR that is not at the centre of rotation. That is now the wrong knob - the offset is applied as `lidar.mount_offset_x`, and `params.py` explicitly says `footprint_offset_x` must stay at zero. **Fixed.** | A |
| 3 | The launch files could not follow a simulator's clock. A `use_sim_time` argument was added, defaulting to `false`, so behaviour on the robot is unchanged. | required for simulation |
| 4 | `robot_radius: 0.22` (from the robot's `nav2_params.yaml`) is **smaller than the chassis half-diagonal, 0.260 m**, so the optional circular footprint understates the real corners by 40 mm. The rectangular footprint is the default and is exact, so this is latent - but `use_circular_footprint: true` is not a safe substitute on this chassis. | B |
| 5 | `colcon build --symlink-install`, which the README instructs, fails against setuptools >= 80 ("option --editable not recognized"). Environment-specific, not a repository fault; a plain `colcon build` works. | C |

---

## 6. Physical implementability

Checked against the published hardware, not against the simulation.

| Item | Code | Hardware | Verdict |
|---|---|---|---|
| Footprint | 0.375 x 0.360 m | 375 x 360 mm | **exact** |
| LiDAR offset | 0.085 m fwd, 0.206 m up | 8.5 cm fwd, 20.6 cm up | **exact** |
| Max forward speed | 0.22 m/s | 1.0 m/s | within, 22% |
| Max angular speed | 1.0 rad/s | 2.0 rad/s clamp | within |
| Max linear accel | 0.35 m/s^2 | 0.68 m/s^2 from the firmware ramp | within |
| Max linear decel | 0.60 m/s^2 | 0.68 m/s^2 | within, only 12% margin |
| Reverse speed | 0.12 m/s | in-place rotation supported | within |
| LiDAR range gate | 0.1-12 m | RPLidar C1 12 m | matches |
| Scan rate assumption | 10 Hz control | LiDAR 10 Hz | matches |
| Control loop cost | 1.0 ms mean, 1.9 ms worst | Pi 5 is slower than this host | large margin |

The braking guarantee `v <= sqrt(2 a (d - margin))` uses
`max_linear_decel = 0.60 m/s^2`, and the firmware ramp allows 0.68 m/s^2. The
margin is real but thin, and the ramp figure is a *commanded* RPM limit, not a
measured deceleration on a floor. On carpet or a dusty floor the achievable
figure will be lower, which makes the braking guarantee optimistic.

### The interface gap that will stop the robot moving

The navigation node publishes `/cmd_vel_nav`. According to VEEROBOT's node
graph, the chain that actually reaches the wheels is:

```
teleop -> /cmd_vel_joy -> cmd_vel_gate -> /cmd_vel -> lyra_bridge -> STM32
                              ^
                              +-- gated on /lyra/armed
```

Two things follow, and neither is visible in simulation:

1. **Nothing subscribes to `/cmd_vel_nav` on the stock robot.** `lyra_bridge`
   consumes `/cmd_vel`. Either a mux must be configured to forward
   `/cmd_vel_nav`, or the node must be launched with
   `cmd_vel_topic:=/cmd_vel`.
2. **The motors must be armed.** `cmd_vel_gate` passes commands only while
   `/lyra/armed` is true, and arming is a `std_srvs/Trigger` call to
   `/lyra/arm` - on the real robot, holding the controller's LB button. This
   package never calls `/lyra/arm`, and the Lyra firmware safe-stops 500 ms
   after commands stop. A correct, well-behaved run can therefore produce no
   motion at all, with nothing in the logs to explain it.

Neither is a defect in the navigation logic. Both are integration steps that
must be settled before the first physical run, and the second one is the more
likely to waste an afternoon.

### Skid-steer, not differential drive

The robot is a **4-wheel skid-steer**. The code commands it as a differential
drive, which is correct - `linear.x` and `angular.z` are exactly what
`lyra_bridge` converts to four wheel speeds. But a skid-steer scrubs its tyres
when turning, so:

* Its effective track exceeds the geometric 0.29 m, and a commanded yaw rate
  produces less rotation than the model predicts. VEEROBOT quantifies this:
  **+/-5-10% angular odometry error**, against +/-3-5% linear.
* The simulation uses the geometric track and so turns *better* than the real
  robot will.
* Rotation accuracy matters most to the recovery behaviour, which is
  open-loop-timed (`recovery_rotate_time`, `recovery_rotate_speed`) rather than
  closed on measured yaw. A rotate that under-delivers by 10% will under-rotate
  by 10%, every time, in the situation where the robot is already stuck.

`goal.align_final_yaw` is `false` by default, which avoids the worst of this.

---

## 7. Status

Demonstrated in simulation, with the real code:

* The full ROS 2 system launches; nodes, topics, types, parameters and the TF
  tree are consistent, with no type or QoS mismatches.
* The node consumes the simulated RPLidar C1, wheel odometry and TF, and drives
  the simulated robot to goals it can reach.
* Goals are reached in the odometry frame and in the map frame under AMCL,
  including from offset start poses.
* Static obstacles, a corridor, a 1.2 m doorway and a cluttered room are all
  handled - the last two only with more planner lookahead than ships by default.
* The safety supervisor, recovery behaviour, CSV logging and `analyze_log.py`
  all work against live ROS.
* The control loop has two orders of magnitude of timing headroom.

Not established:

* Anything about the physical robot. No hardware was involved.
* Whether `/cmd_vel_nav` reaches the wheels, and whether arming is handled.
* Real LiDAR noise, wheel slip, floor friction, motor response, battery sag.
* Real braking deceleration, on which the braking guarantee depends.
* SLAM mapping.
* Whether the parameter change that fixes the stalls is acceptable at the
  speeds it implies near obstacles - that is a judgement about the real robot
  in a real room.
