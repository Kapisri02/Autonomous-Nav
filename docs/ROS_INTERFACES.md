# ROS interfaces, frames and assumptions

What this package expects from the rest of the system, and what it provides.
Anything listed as an assumption has **not** been verified against a running
robot. The package has since been run against a Gazebo Harmonic simulation of
the BeetleBot built to VEEROBOT's published dimensions - see
[SIMULATION.md](SIMULATION.md) for what that did and did not establish.

## Subscriptions

| Topic | Type | QoS | Required | Notes |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | best effort, depth 5 | **yes** | RPLiDAR C1. No motion is commanded without it. |
| `/odom` | `nav_msgs/Odometry` | best effort, depth 5 | yes by default | Pose fallback and measured velocity. Set `safety.require_odom: false` to make it optional. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | reliable, depth 10 | yes | What RViz's "2D Goal Pose" tool publishes. |

Topic names are parameters (`scan_topic`, `odom_topic`, `goal_topic`).

## Publications

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/Twist` | The safe velocity command. Topic set by `cmd_vel_topic`; the default matches the existing baseline, which feeds your `cmd_vel_mux`. |
| `/beetlebot_nav/status` | `std_msgs/String` | One human-readable line per cycle. |
| `/beetlebot_nav/goal_reached` | `std_msgs/Bool` | `true` once, when the goal is reached. |

Only `linear.x` and `angular.z` are set - the BeetleBot is a differential drive.

## Transforms

| Transform | Used for | If missing |
|---|---|---|
| `map -> base_link` | The robot's pose, so a goal picked on the map is meaningful | Falls back to raw `/odom` and warns once. Navigation still works relative to the start pose. |

`global_frame` and `base_frame` are parameters. A goal in a frame that cannot be
resolved is **refused with an error** rather than accepted and driven to - a goal
silently interpreted in the wrong frame would send the robot to the wrong place.

## Verified robot facts

| Fact | Value | Source |
|---|---|---|
| `robot_radius` | 0.22 m | the robot's own `nav2_params.yaml` |
| Chassis | 0.375 m x 0.360 m | official VEEROBOT documentation |
| Compute | Raspberry Pi 5 | VEEROBOT documentation |
| LiDAR | RPLiDAR C1, 360 degrees, 12 m, 10 Hz, ~0.5 deg | VEEROBOT documentation |
| LiDAR mounting | 8.5 cm forward of centre, 20.6 cm above floor | VEEROBOT documentation (two pages) |
| Drive | 4-wheel skid-steer | VEEROBOT documentation |
| Chassis, weight | 375 x 360 x 245 mm, ~2.2 kg | VEEROBOT documentation |
| Max speed / clamp | 1.0 m/s; firmware clamps 1.0 m/s and 2.0 rad/s | VEEROBOT documentation |
| Wheels | 130 mm diameter, 290 mm track, 181 mm wheelbase | VEEROBOT documentation |
| ROS | 2 Jazzy | VEEROBOT documentation |

The rectangular footprint is used by default because the chassis is nearly
square and the exact rectangle costs no more to evaluate than a disc. Set
`robot.use_circular_footprint: true` to use the 0.22 m circle Nav2 itself uses.

## Assumptions still to confirm on the robot

1. ~~The LiDAR frame is the robot's centre of rotation.~~ **Resolved - this is
   no longer an assumption.** VEEROBOT documents the RPLiDAR C1 as 8.5 cm
   forward of the chassis centre and 20.6 cm above the floor, on two separate
   pages (Hardware Familiarization; Sensor Data Visualization). Those are the
   values in `lidar.mount_offset_x` / the 0.206 m height, and the scan is
   translated into the chassis frame by that offset.

   Note the knob: it is **`lidar.mount_offset_x`**, not
   `robot.footprint_offset_x`. Earlier revisions of this document named the
   latter; that is wrong, and `params.py` explains why - displacing the
   footprint models the robot as rotating about its LiDAR instead of its
   chassis centre, which measurably degraded its ability to turn past an
   obstacle.
2. **`/scan` angles follow the ROS convention** (0 straight ahead, positive to
   the left). The preserved baseline assumes the same thing and works, which is
   good evidence, but it has not been confirmed directly.
3. **`/cmd_vel_nav` reaches the wheels** through your existing `cmd_vel_mux`
   with nothing else fighting it for priority.
4. **Odometry is published at 10 Hz or better.** Slower than
   `safety.odom_timeout` (0.6 s) and the watchdog will stop the robot.
5. **Scan timestamps are populated.** If the driver publishes a zero stamp, the
   node substitutes its own clock, which weakens the staleness check.

## Deliberate design limits

- **No global planner.** The robot steers toward the goal with local avoidance.
  A route that must initially lead away from the goal will not be found. The fix,
  if physical testing calls for it, is A* over the existing `/map`.
- **No IMU subscription.** Odometry already carries what the navigation needs.
  The parameters exist to add it without restructuring anything.
- **Moving obstacles are predicted with constant velocity** over a short horizon.
  A person changing direction sharply is handled by slowing and re-deciding at
  10 Hz, not by predicting the turn.
