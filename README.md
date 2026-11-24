# UWB-IMU Sandbox

A ROS 2 Jazzy + Gazebo Harmonic package for testing indoor (GPS-denied)
localization with UWB and an IMU. It has:

- **A UWB plugin.** The same plugin works as an anchor or a tag. The tag ranges to the anchors one after the other, like the real tag, and prints the same serial line. Range error, signal fields, dropouts and blocked signals come from real measurements.
- **An IMU plugin.** It takes the exact motion from Gazebo and adds the measured jitter, sample timing and output resolution. It also adds a new random turn-on bias every time it starts.
- **Built-in Gazebo worlds.** The default is an indoor warehouse (12 x 10 m): rack rows, columns, pallets, a conveyor and four anchors on 4 m poles. A smaller room with two pillars is included as `uwb_room.sdf` (`world:=uwb_room.sdf`).
- **ROS 2 nodes:** `uwb_node`, `imu_node`, an empty `algorithm_node` for your own algorithm, a demo driver and a self-check.

No localization algorithm or ML model is included. You bring your own.

```
uwb_imu_sandbox_ws/src/
  uwb_imu_sandbox_msgs/     UwbRange.msg, UwbTruth.msg
  uwb_imu_sandbox_gazebo/   plugins (C++), world, robot, sealed sensor models, replay tools, unit tests
  uwb_imu_sandbox/          nodes, bridge config, launch file
```

## 1. Install and build

You need Ubuntu 24.04 with ROS 2 Jazzy. Gazebo Harmonic comes with `ros-jazzy-ros-gz`.

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-rosbag2-storage-mcap python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash

cd ~/uwb_imu_sandbox_ws            # the folder that contains src/
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

Run `source install/setup.bash` in every new terminal. It also tells Gazebo where the plugins, the world and the model files are.

## 2. Test it, step by step

**Step 1: unit tests (about 1 s).**

```bash
colcon test && colcon test-result --verbose
```

You should see `35 tests, 0 errors, 0 failures`. They check the obstacle geometry, the sealed model files, the UWB and IMU models, and the serial-line parser.

**Step 2: start the simulation.**

```bash
ros2 launch uwb_imu_sandbox sandbox.launch.py
```

The Gazebo window opens with the warehouse. Drive the robot with the **Drive the robot** panel on the right: use the arrow buttons, or the keyboard tab.

To let it drive a loop on its own, start it like this:

```bash
ros2 launch uwb_imu_sandbox sandbox.launch.py demo:=true
```

The loop runs through the main aisle, the dock area and past the rack ends, in and out of the racks' shadows (it is made for the warehouse; for another world, pass your own `waypoints`). Add `gui:=false` to run without a window.

**Step 3: health check (second terminal, while step 2 runs).**

```bash
ros2 run uwb_imu_sandbox self_check --ros-args -p duration:=30.0
```

It listens for 30 s and must end with `ALL CHECKS PASSED`. It checks:

- **IMU:** rate near 100 Hz, gravity seen, frame name.
- **UWB:** every anchor reports at about 6 Hz, the network has 4 anchors, no NaN values.
- **Range error:** LOS error looks like a DW1000 (median between −15 and +35 cm), and NLOS ranges appear.
- **Other topics:** `/uwb/ranges` and the ground truth.

Run it with `demo:=true` so the robot moves through both LOS and NLOS spots.

**Step 4: look at the data.**

```bash
ros2 topic hz /imu                 # ~100 Hz
ros2 topic echo /uwb/range         # one message per ranging result
ros2 topic echo /uwb/ranges        # [a1, a2, a3, a4], NaN when an anchor is stale
ros2 topic echo /uwb/truth         # true distance, LOS or not, why a slot failed
ros2 topic echo /sandbox/uwb/serial   # the raw tag line, same format as the real tag
```

To see NLOS, drive the robot to around (-3, 1.4), next to the north rack row. Anchor 1 is then behind the rack and its goods: `/uwb/truth` shows `line_of_sight: false` and the `quality` in `/uwb/range` drops. Right against a rack (about 0.5 m from it) the path crosses more than 1 m of material and that anchor gives no reading at all.

**Step 5: send the UWB data to an outside program (for example an ML pipeline).**

```bash
ros2 launch uwb_imu_sandbox sandbox.launch.py udp_forward:=true udp_port:=9999
```

Every range then also goes out as one JSON datagram to `127.0.0.1:9999`:
`{"tag": anchor_id, "net": .., "dist": .., "rx": .., "fp": .., "q": .., "delta": .., "t": sim_time}`.

Quick listener:

```bash
python3 -c "import socket;s=socket.socket(2,2);s.bind(('127.0.0.1',9999));print(s.recv(4096))"
```

**Step 6: record a dataset.**

```bash
ros2 bag record -s mcap /uwb/range /uwb/truth /imu /sandbox/imu/truth /ground_truth/odom
```

**Step 7: check the IMU bias changes at every start.** Stop and start the launch. Compare `/imu` with `/sandbox/imu/truth` (same values without any error) while the robot stands still. The difference in `linear_acceleration` is the turn-on bias, and it is different every time.

**Step 8 (optional): run the models without Gazebo.** This is useful for quick experiments.

```bash
# trajectory.csv: header, then rows "t,x,y,z" of the tag
ros2 run uwb_imu_sandbox_gazebo uwb_replay \
  install/uwb_imu_sandbox_gazebo/share/uwb_imu_sandbox_gazebo/worlds/uwb_warehouse.sdf \
  install/uwb_imu_sandbox_gazebo/share/uwb_imu_sandbox_gazebo/models/uwb_imu_sandbox_data/uwb_model.bin \
  trajectory.csv 42 > sim_ranges.csv
```

`imu_replay` does the same for the IMU. Its input is rows of `t,vx,vy,vz,wx,wy,wz,qw,qx,qy,qz`.

## 3. Topics

| Topic | Type | What it is |
|---|---|---|
| `/uwb/range` | `uwb_imu_sandbox_msgs/UwbRange` | one ranging result: range [m] plus `rx_level`, `fp_level`, `quality`, `delta_level` |
| `/uwb/ranges` | `std_msgs/Float64MultiArray` | latest range per anchor, 20 Hz, NaN if older than 0.8 s (best-effort QoS) |
| `/imu` | `sensor_msgs/Imu` | ~100 Hz, frame `base_link` |
| `/uwb/truth` | `uwb_imu_sandbox_msgs/UwbTruth` | ground truth per ranging slot (for scoring only) |
| `/sandbox/imu/truth` | `sensor_msgs/Imu` | IMU without any error (for scoring only) |
| `/ground_truth/odom` | `nav_msgs/Odometry` | exact robot pose (for scoring only) |
| `/cmd_vel` | `geometry_msgs/Twist` | drive the robot |
| `/sandbox/uwb/serial` | `std_msgs/String` | raw tag line `[net,anchor,range,rx,fp,quality,delta]` |

The four UWB signal fields are **normalised, unitless values (0..100)**, not dBm. They keep the order of the real quantity: higher `rx_level` means a stronger signal, and `quality` drops when a signal is blocked. They also keep how the fields behave with distance and obstacles, so an LOS/NLOS classifier or range filter can be trained and tested on them. In our tests, a classifier trained only on simulated fields got about 97% of real held-out measurements right.

## 4. Use it for your algorithm

Fill in `uwb_imu_sandbox/algorithm_node.py`. It already subscribes to `/uwb/range`, `/uwb/ranges` and `/imu`. You can also write your own node, or use UDP (step 5).

Score your estimate against `/ground_truth/odom`.

Anchor positions (world frame, metres) are 1 (3.425, 5.136, 1.02), 2 (−3.998, 5.136, 1.02), 3 (−3.998, −5.136, 1.02) and 4 (3.425, −5.136, 1.02). The tag antenna is 0.50 m above the floor.

## 5. Plugin settings

Put the anchors and tags in any world with these plugin blocks.

**UWB anchor**

```xml
<plugin filename="UwbDevice" name="uwb_imu_sandbox::UwbDevice">
  <role>anchor</role>
  <id>5</id>
</plugin>
```

**UWB tag:** see `models/sandbox_robot/model.sdf` for a full example.

| Parameter | Default | Meaning |
|---|---|---|
| `role` | `tag` | `anchor` or `tag` |
| `id` | 1 | device id (anchor ids must be unique) |
| `link_name` | canonical link | link the antenna is on |
| `antenna_offset` | 0 0 0 0 0 0 | antenna pose on that link |
| `max_range` | 12.0 | beyond this the tag cannot hear an anchor [m] |
| `max_material` | 1.0 | more obstacle thickness than this on the path = no signal [m] |
| `nlos_cell_size` | 0.25 | NLOS behaviour is fixed per floor cell of this size [m] (0 = smooth blend) |
| `multipath_seed` | 0 | changes which cell gets which NLOS behaviour |
| `antenna_delay_spread` | 0.03 | each anchor gets its own constant range offset, drawn per run [m] |
| `antenna_delay_offset` | 0.0 | constant offset to match your own hardware calibration [m] |
| `join_delay`, `leave_delay` | 0.5, 1.0 | time to join / leave the tag's network [s] |
| `seed` | -1 | random seed (-1 = new every run) |
| `topic`, `truth_topic` | `uwb/tag<id>/serial`, `uwb/tag<id>/truth` | Gazebo topics |

Obstacles are taken from the collision shapes in the world, including moving ones. Box, cylinder, sphere, capsule and ellipsoid shapes block the radio; mesh collisions are ignored, with a warning. A tag never blocks itself, and an anchor never blocks itself.

**IMU** (`filename="RealisticImu"`)

| Parameter | Default | Meaning |
|---|---|---|
| `link_name` | canonical link | link the IMU is on |
| `imu_offset` | 0 0 0 0 0 0 | IMU pose on that link |
| `topic`, `truth_topic` | `imu/raw`, `imu/truth` | Gazebo topics |
| `frame_id` | `base_link` | frame in the message |
| `update_rate` | 100 | driver read-out rate [Hz] |
| `seed` | -1 | -1 = new random turn-on bias at every start |
| `bias_reseed_period` | 600 | draw a new bias after this long [s] (0 = never) |
| `bias_transition_time` | 5 | how fast it moves to the new bias [s] |
| `bias_scale`, `noise_scale` | 1.0 | scale the bias spread / the jitter |
| `enable_noise`, `enable_bias`, `enable_quantisation`, `enable_sample_hold` | true | switch single effects off |

## 6. Good to know

- **Calibration offset.** The LOS range offset is that of the tag the models were measured with: from about −7 cm at 0.5 m to about +20 cm at 5 to 12 m. Another DW1000 pair can sit 10 cm or more away from that. Use `antenna_delay_offset` to match yours.
- **Multipath depends on the spot.** NLOS errors change from spot to spot, as in the measurements, so one spot behind a pillar can be much worse than the one next to it.
- **What is not modelled.** People and the robot's own body do not block the radio.
- **GUI speed.** With software rendering (no GPU), the GUI can run below real time. Simulated time is always correct; use `gui:=false` for long runs.
- **Rates 2× or 3× too high, robot jumping around?** An old Gazebo server is still running from an earlier launch. Each launch now runs in its own Gazebo partition, so old servers cannot leak in, but stop them anyway with `pkill -f "gz sim"`. The self-check flags this as `only one simulation running`.
- **Sealed model files.** `uwb_model.bin` and `imu_model.bin` hold fitted tables only, not the measurements. Do not edit them; the plugins check them on load.
