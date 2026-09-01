# IPP (Informative Path Planning)

This package plans, follows, and controls a BlueROV2 exploring an IR field in the institute's water tank (2 m by 4 m by default, configurable in `mapping_params.yaml`). It replans in a receding-horizon loop using a RIG-tree (Hollinger & Sukhatme, IJRR 2014).

## Dependencies

The following packages must be cloned into the same workspace as `ipp`.

| Package | Purpose |
|---|---|
| `ipp_interfaces` | Custom ROS message types for this package |
| `scalar_field_interfaces` | ROS messages and services for asking about the scalar field (in this thesis, the IR field read by the photodiode array) |
| `scalar_field_sim` | From HippoCampus Robotics. Simulates a 2D IR field (fake sources, walls that block the signal, noise) and answers measurement requests. Stands in for the real sensor when running in simulation |
| `scalar_field_belief` | Keeps a running estimate of the field. It listens to measurements, fits a Gaussian Process (GP) model with GPyTorch, and answers questions about the estimated value and uncertainty at any point |
| `ipp_rviz_visuals` | Draws the water tank walls and the BlueROV mesh in RViz. Needed by both `simulation.launch.py` and `experiment.launch.py` |
| `hippo_common` | HippoCampus Robotics' shared robot framework. Must be this fork, not the plain HippoCampus Robotics version. It adds the `ir_sensor` position offset (see "Measurements" below) that `ir_measurement_bridge.py` needs on real hardware |
| `rviz_2d_overlay_plugins` | Third-party RViz plugins from Team Spatzenhirn (University of Ulm), unrelated to HippoCampus Robotics. Provides the pie chart and text overlays. Needed so `rviz.rviz` and `rviz_experiment.rviz` load without errors |

```bash
cd <workspace>/src
git clone https://github.com/Latfoo/ipp_interfaces.git
git clone https://github.com/Latfoo/scalar_field_interfaces.git
git clone https://github.com/HippoCampusRobotics/scalar_field_sim.git
git clone https://github.com/Latfoo/scalar_field_belief.git
git clone https://github.com/Latfoo/ipp_rviz_visuals.git
git clone https://github.com/Latfoo/hippo_common.git
git clone https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git
```

Other packages this depends on, such as `hippo_control_msgs`, `hippo_sim`, and `tf_transformations`, are regular ROS packages and do not need to be cloned by hand. They get pulled in by rosdep instead.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Then rebuild the workspace.

## Run (simulation)

**Terminal 1**. Start Gazebo, spawn the robot, and start all IPP nodes.

```bash
ros2 launch ipp simulation.launch.py
```

Optional arguments.

| Argument | Default | Description |
|---|---|---|
| `vehicle_name` | `bluerov00` | Name of the spawned BlueROV in Gazebo. Also used as the ROS namespace for all topics and services. The BlueROV model itself is fixed and cannot be changed here |
| `scenario` | _(empty)_ | Obstacle layout to load (e.g. `scenario_1`). Leave empty for a free (empty) tank |
| `planner` | `rig` | Which planning algorithm to run, `rig` (RIG-tree), `lawnmower` (a fixed back-and-forth coverage pattern, used as a non-informative baseline), or `random` (RIG-tree without using variance information, i.e. a random walk). Sets the `planner_type` parameter of `path_planner.py`. There is only ever one planner node running |
| `start_gui` | `false` | Show the Gazebo window. Runs headless (no window) by default |

```bash
# for instance, with scenario 1 obstacles and Gazebo GUI
ros2 launch ipp simulation.launch.py scenario:=scenario_1 start_gui:=true

# run the lawnmower baseline instead of RIG-tree
ros2 launch ipp simulation.launch.py planner:=lawnmower

# run the random-walk baseline (RIG-tree without variance information)
ros2 launch ipp simulation.launch.py planner:=random
```

RViz launches automatically alongside Gazebo.

**Terminal 2**. Start and stop the planner.

```bash
ros2 service call /bluerov00/path_planner/start std_srvs/srv/Trigger {}
ros2 service call /bluerov00/path_planner/stop std_srvs/srv/Trigger {}
```

Start and stop can be called repeatedly to pause and resume the mission. If `vehicle_name` was changed, replace `bluerov00` in the service calls accordingly.

Before starting a mission, the robot can be driven to its configured start pose first. This service only works while the planner is `IDLE` and a pose estimate is available.

```bash
ros2 service call /bluerov00/path_planner/move_to_start std_srvs/srv/Trigger {}
```

## Run (real hardware)

Real-hardware runs use `experiment.launch.py` instead of `simulation.launch.py`. There is no Gazebo to start, so it is a single command.

```bash
ros2 launch ipp experiment.launch.py
```

One important difference is that `vehicle_name` defaults to `bluerov01` here, not `bluerov00` like in simulation. All the other arguments work the same as in `simulation.launch.py` (`scenario`, `planner`, `rng_seed`, `planning_horizon`, `planning_time_s`, `roi_adaptive_enabled`, `roi_threshold`, `roi_beta`), just without `start_gui`, since there is no Gazebo.

Start, stop, and move-to-start work exactly the same as in simulation, just under the `bluerov01` namespace instead of `bluerov00`.

```bash
ros2 service call /bluerov01/path_planner/start std_srvs/srv/Trigger {}
ros2 service call /bluerov01/path_planner/stop std_srvs/srv/Trigger {}
ros2 service call /bluerov01/path_planner/move_to_start std_srvs/srv/Trigger {}
```

## Measurements (sim vs. real hardware)

The robot explores a scalar IR field. Whenever `path_planner.py` decides it needs a new measurement, it calls the `trigger_ir_measurement` service. Whatever answers that call publishes the result on the `ir_measurement` topic (a `ScalarMeasurement` message with `frame_id = 'map'`). `scalar_field_belief_node.py` only listens on `ir_measurement` and updates its GP estimate from whatever arrives there. It never needs to know whether the measurement came from simulation or real hardware.

**In simulation**, `ir_measurement_sim_node.py` (package `scalar_field_sim`) answers `trigger_ir_measurement`. It takes the robot's current position, asks the fake scalar field for a value there, and publishes the result. Started automatically by `simulation.launch.py`.

**On real hardware**, `ir_measurement_bridge.py` (this package) answers `trigger_ir_measurement` instead. The real sensor is a 9-channel photodiode array. A separate node, `ads1115_publisher`, publishes each channel as a `std_msgs/Float32` at 50 Hz, on topics `/photodiode_array/diode_1` through `/photodiode_array/diode_9` (not namespaced under the vehicle). That is much faster than the GP can use. The planner only wants a new measurement every `measurement_period_s` (0.5 s by default), i.e. roughly every 25th sample.

To handle this, `ir_measurement_bridge.py` keeps a short rolling buffer per channel (the last `fusion_window_s`, 0.1 s by default, about 5 samples per channel). When triggered, it does the following.

1. averages each channel's recent buffer,
2. combines the 9 channel averages into one value (`fuse_ir_channels()`, currently a plain average),
3. publishes the result on `ir_measurement`.

It also needs the robot's current pose, read from `vision_pose_cov_bridged`. Started automatically by `experiment.launch.py`. The `ir_channel_topics` parameter already defaults to the real topic names above. The channel-fusion formula is still just a plain average, and may need to become something more specific to the thesis's field model.

Only one of these two nodes ever runs at once. Each is started by a different launch file, so there is never more than one publisher on `ir_measurement`.

On real hardware, the IR sensor is mounted a bit away from the point whose pose the vehicle actually tracks. `ir_measurement_bridge.py` corrects for this with an `ir_sensor.x/y/z` offset, defined in `transformations_bluerov_default.yaml` (package `hippo_common`). It rotates this offset into the robot's current orientation and adds it to the tracked pose, so the position reported to the GP is the sensor's real position, not the tracked point.

This offset only affects measurements. `path_follower.py` and `position_controller.py` ignore it completely. They always drive the tracked point straight to each waypoint. In simulation, `ir_measurement_sim_node.py` samples exactly at the tracked pose, since the offset is zero there anyway.

## Scenarios

Obstacle layouts live in YAML files under `config/`. Each file lists polygon obstacles as corner coordinates in metres. Pass the scenario name without the `.yaml` extension (see examples above). Without a scenario, the robot operates in a free (empty) tank.

```text
config/scenario_1.yaml
config/scenario_2.yaml
config/scenario_3.yaml
```

> **Note.** `scalar_field_sim` keeps its own separate map (bounds and walls, configured in a TOML file) and does not read this package's occupancy grid. So if you add a physical obstacle, you currently have to describe it in two places, once here (for collision avoidance) and once in the matching `scalar_field_sim` scenario (so the field is attenuated correctly).

## Package structure

```text
nodes/
  rig_tree.py                 RIG-tree algorithm (Hollinger & Sukhatme 2014, Alg. 2)
  lawnmower.py                 serpentine coverage path generator (non-informative baseline)
  path_planner.py             planner node.
                               planner_type='rig' is receding-horizon RIG-tree,
                               'lawnmower' is one-shot coverage sweep, or 'random'
                               is receding-horizon RIG-tree without variance information
  path_follower.py             pure-pursuit waypoint follower
  mapper.py                    builds occupancy grid from obstacles, inflates obstacles by 30 cm
  obstacle_publisher.py       loads a scenario YAML and publishes polygon obstacles at 10 Hz
  position_controller.py      PD control in x/y, PID control in z
  yaw_controller.py            P-control for yaw
  vision_pose_bridge.py       republishes ground_truth/odometry (nav_msgs/Odometry)
                               as vision_pose_cov_bridged (PoseWithCovarianceStamped),
                               real hardware only, started by experiment.launch.py
  ir_measurement_bridge.py    turns the 9 real IR sensor topics into one
                               ir_measurement + trigger_ir_measurement pair,
                               same job ir_measurement_sim_node.py does in sim,
                               real hardware only, started by experiment.launch.py
launch/
  simulation.launch.py        Gazebo plus all IPP nodes (sim only)
  ipp.launch.py                IPP nodes only
  experiment.launch.py        real hardware only, IPP nodes plus vision_pose_bridge.py,
                               ir_measurement_bridge.py, and scalar_field_belief_node.py
config/
  mapping_params.yaml          tank dimensions and grid resolution
  scenario_1.yaml               obstacle layout 1
  scenario_2.yaml               obstacle layout 2
  scenario_3.yaml               obstacle layout 3
  rviz.rviz                     RViz config for simulation
  rviz_experiment.rviz         RViz config for real-robot experiments
  ir_sensor_offset_zero.yaml   zeroes the ir_sensor.x/y/z offset,
                               sim only, see "Measurements" above
```

RViz visual markers (water tank walls, BlueROV mesh) are published by the separate `ipp_rviz_visuals` package (`src/robot_marker_publisher.cpp` inside that package). That package is launched automatically by both `simulation.launch.py` and `experiment.launch.py`.

> **Note.** On real hardware, two different nodes publish to `vision_pose_cov`, `qualisys_bridge` (motion capture, wrong message type for the IPP nodes) and `vision_pose_bridge.py` (this package, converts `ground_truth/odometry` into the right type). If both published to the same topic, every subscriber would receive a mix of both. To avoid this, `experiment.launch.py` remaps all IPP nodes from `vision_pose_cov` to `vision_pose_cov_bridged`, the topic `vision_pose_bridge.py` actually publishes on. This remap only happens in `experiment.launch.py`. In simulation, `vision_pose_cov` has only one publisher, so nothing needs remapping.
