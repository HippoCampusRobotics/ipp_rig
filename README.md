# IPP - Informative Path Planning

This package implements informative path planning for a BlueROV2 operating in a bounded water area. The robot explores the area autonomously, replanning its path in a receding-horizon loop using a RIG-tree (Hollinger & Sukhatme, IJRR 2014). The workspace dimensions default to the institute's water tank but can be configured via `mapping_params.yaml`.

## Dependencies

The following packages must be cloned into the same workspace:

| Package | Purpose |
|---|---|
| `ipp_interfaces` | Custom message types for this package |
| `scalar_field_interfaces` | ROS messages and services for scalar field queries |
| `scalar_field_sim` | Simulates a 2D scalar field (Gaussian sources, wall attenuation, noise) and serves measurements on request. Acts as a stand-in for the real sensor |
| `scalar_field_belief` | Maintains a GP belief over the scalar field: subscribes to measurements, fits an exact GPyTorch GP, and serves posterior mean and variance queries |
| `ipp_rviz_visuals` | Publishes RViz marker arrays for the water tank walls and the BlueROV mesh. Required by both `simulation.launch.py` and `experiment.launch.py` |

```bash
cd <workspace>/src
git clone  ...
```

Then install any remaining ROS dependencies and rebuild.

## Run (simulation)

**Terminal 1** - start Gazebo + Spawn Robot and all IPP nodes:

```bash
ros2 launch ipp simulation.launch.py
```

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `vehicle_name` | `bluerov00` | Names the spawned BlueROV in Gazebo and sets the ROS namespace for all topics and services (the BlueROV model is hardcoded) |
| `scenario` | _(empty)_ | Obstacle layout to load (e.g. `scenario_1`). Leave empty for a free tank. |
| `planner` | `rig` | `rig` (RIG-tree), `lawnmower` (non-informative coverage baseline), or `random` (RIG-tree without variance information, i.e. a random walk). Sets `path_planner.py`'s `planner_type` parameter; there is only ever one planner node. |
| `start_gui` | `false` | Show the Gazebo GUI. Runs headless by default. |

```bash
# for instance with scenario 1 obstacles and Gazebo GUI:
ros2 launch ipp simulation.launch.py scenario:=scenario_1 start_gui:=true

# run the lawnmower baseline instead of RIG-tree:
ros2 launch ipp simulation.launch.py planner:=lawnmower

# run the random-walk baseline (RIG-tree without variance information):
ros2 launch ipp simulation.launch.py planner:=random
```

RViz launches automatically alongside Gazebo.

**Terminal 2** - start and stop the planner:

```bash
ros2 service call /bluerov00/path_planner/start std_srvs/srv/Trigger {}
ros2 service call /bluerov00/path_planner/stop std_srvs/srv/Trigger {}
```

Start and stop can be called repeatedly to pause and resume the mission. If `vehicle_name` was changed, replace `bluerov00` in the service calls accordingly.

Before starting a mission the robot can be driven to its configured start pose. The service is accepted only when the planner is in the `IDLE` state and a pose estimate is available:

```bash
ros2 service call /bluerov00/path_planner/move_to_start std_srvs/srv/Trigger {}
```

## Run (real hardware)

Real-hardware runs use `experiment.launch.py` instead of `simulation.launch.py`. There is no Gazebo to start, so it is a single command:

```bash
ros2 launch ipp experiment.launch.py
```

Important: here `vehicle_name` defaults to `bluerov01`, not `bluerov00` like in simulation. All the other arguments work the same as in `simulation.launch.py` (`scenario`, `planner`, `rng_seed`, `planning_horizon`, `planning_time_s`, `roi_adaptive_enabled`, `roi_threshold`, `roi_beta`), just without `start_gui`, since there is no Gazebo.

Start, stop, and move-to-start work exactly the same as in simulation, just under the `bluerov01` namespace instead of `bluerov00`:

```bash
ros2 service call /bluerov01/path_planner/start std_srvs/srv/Trigger {}
ros2 service call /bluerov01/path_planner/stop std_srvs/srv/Trigger {}
ros2 service call /bluerov01/path_planner/move_to_start std_srvs/srv/Trigger {}
```

## Measurements (sim vs. real hardware)

The robot is exploring a scalar field (for example light or temperature). It takes a measurement whenever `path_planner.py` decides it needs one, by calling the `trigger_ir_measurement` service. Whatever answers that call is expected to publish the result on the `ir_measurement` topic (`ScalarMeasurement`, `frame_id = 'map'`). `scalar_field_belief_node.py` just listens on `ir_measurement` and updates its GP belief from whatever arrives there - it never needs to know whether the measurement came from sim or real hardware.

**In simulation:** `ir_measurement_sim_node.py` (package `scalar_field_sim`) answers `trigger_ir_measurement`. It takes the robot's current position, asks the fake scalar field for a value there, and publishes the result. Started by `simulation.launch.py`.

**On real hardware:** `ir_measurement_bridge.py` (this package) answers `trigger_ir_measurement` instead. The real sensor is a 9-channel photodiode array (`ads1115_publisher` node, publishing `std_msgs/Float32` at 50 Hz on `/photodiode_array/diode_1` through `/photodiode_array/diode_9`, not namespaced under the vehicle) that streams continuously and much faster than the GP can use (the planner only wants a new measurement every `measurement_period_s`, 0.5 s by default, i.e. every 25th sample). So this node keeps a short rolling buffer per channel (the last `fusion_window_s`, 0.2 s by default, about 10 samples per channel) and, when triggered: averages each channel's recent buffer, combines the 9 channel averages into one value (`fuse_ir_channels()`, currently a plain average), and publishes it on `ir_measurement`. It also needs the robot's current pose, read from `vision_pose_cov_bridged`. Started by `experiment.launch.py`. The `ir_channel_topics` parameter defaults to the confirmed real topic names above; the channel-fusion formula is still just a plain average and may need to become something more specific to the thesis's field model.

Only one of these two nodes ever runs at a time, since each is started by its own launch file, so there is never more than one publisher on `ir_measurement`.

On real hardware the IR sensor sits a bit off from the point the vehicle's pose is tracked at. `ir_measurement_bridge.py` corrects for this with an `ir_sensor.x/y/z` offset (from `transformations_bluerov_default.yaml`, package `hippo_common`), rotating it into the current orientation and adding it to the tracked pose before publishing a measurement, so the position reported to the GP is the sensor's actual position, not the tracked point. `path_follower.py` and `position_controller.py` are not affected by this offset at all - they always drive the tracked point straight to each waypoint. In simulation, `ir_measurement_sim_node.py` samples exactly at the vehicle's tracked pose with no offset, since the offset is zero there anyway.

## Scenarios

Obstacle layouts are defined in YAML files under `config/`. Each file contains a list of polygonal obstacles with corner coordinates in metres. Pass the scenario name without the `.yaml` extension. Without a scenario the robot operates in a free tank.

```text
config/scenario_1.yaml
config/scenario_2.yaml
config/scenario_3.yaml
```

> **Note:** The `scalar_field_sim` package has its own separate spatial representation (bounds and wall segments configured via a TOML file). It does not read the occupancy grid from this package. If a physical obstacle is present, it currently has to be described in two places independently: here for collision avoidance, and in the `scalar_field_sim` scenario for correct field attenuation.

## Package structure

```text
nodes/
  rig_tree.py            - RIG-tree algorithm (Hollinger & Sukhatme 2014, Alg. 2)
  lawnmower.py           - serpentine coverage path generator (non-informative baseline)
  path_planner.py        - planner node; planner_type='rig' (receding-horizon RIG-tree),
                           'lawnmower' (one-shot coverage sweep), or 'random'
                           (receding-horizon RIG-tree without variance information)
  path_follower.py       - pure-pursuit waypoint follower
  mapper.py              - builds occupancy grid from obstacles; inflates obstacles by 30 cm
  obstacle_publisher.py  - loads a scenario YAML and publishes polygon obstacles at 10 Hz
  position_controller.py - PD control in x/y, PID control in z
  yaw_controller.py      - P-control for yaw
  vision_pose_bridge.py  - republishes ground_truth/odometry (nav_msgs/Odometry)
                           as vision_pose_cov_bridged (PoseWithCovarianceStamped);
                           real hardware only, started by experiment.launch.py
  ir_measurement_bridge.py - turns the 9 real IR sensor topics into one
                           ir_measurement + trigger_ir_measurement pair,
                           same job ir_measurement_sim_node.py does in sim;
                           real hardware only, started by experiment.launch.py
launch/
  simulation.launch.py   - Gazebo plus all IPP nodes (sim only)
  ipp.launch.py          - IPP nodes only
  experiment.launch.py   - real hardware only: IPP nodes plus vision_pose_bridge.py,
                           ir_measurement_bridge.py, and scalar_field_belief_node.py
config/
  mapping_params.yaml    - tank dimensions and grid resolution
  scenario_1.yaml        - obstacle layout 1
  scenario_2.yaml        - obstacle layout 2
  scenario_3.yaml        - obstacle layout 3
  rviz.rviz              - RViz config for simulation
  rviz_experiment.rviz   - RViz config for real-robot experiments
  ir_sensor_offset_zero.yaml - zeroes the ir_sensor.x/y/z offset;
                           sim only, see "Measurements" above
```

RViz visual markers (water tank walls, BlueROV mesh) are published by the separate `ipp_rviz_visuals` package (`src/robot_marker_publisher.cpp`). That package is launched automatically by both `simulation.launch.py` and `experiment.launch.py`.

> **Note:** On real hardware, `vision_pose_cov` has two publishers: `qualisys_bridge` (motion capture, gives the wrong message type for the IPP nodes) and `vision_pose_bridge.py` (this package, converted from `ground_truth/odometry`). Two publishers on one topic would mix both into every subscriber, so `experiment.launch.py` remaps all IPP nodes from `vision_pose_cov` to `vision_pose_cov_bridged` - the topic `vision_pose_bridge.py` actually publishes. This remap only happens in `experiment.launch.py`; in simulation, `vision_pose_cov` has just the one publisher and nothing is remapped.
