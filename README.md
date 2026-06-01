# IPP — Informative Path Planning

RIG-tree based informative path planner for the hippocampus AUV.

## Build

### Personal aliases
```bash
build_ros       # colcon build --symlink-install (Python changes live immediately, no rebuild needed)
rosdep-ros2     # rosdep install --from-paths src (install any missing ROS dependencies)
bcode           # cd ~/b_thesis/ros2 && code . (open workspace in VS Code)
```

> Python files don't need a rebuild after edits — `--symlink-install` keeps them live.
> Rebuild is only needed when adding new files or changing CMakeLists.txt.

## Run (simulation)

```bash
# Terminal 1: Gazebo + IPP nodes
ros2 launch ipp simulation.launch.py

# Terminal 2: start the planner
ros2 service call /bluerov00/path_planner/start std_srvs/srv/Trigger {}

# Terminal 2: stop the planner
ros2 service call /bluerov00/path_planner/stop std_srvs/srv/Trigger {}
```

## Run (real robot)

```bash
ros2 launch ipp ipp.launch.py vehicle_name:=bluerov00 use_sim_time:=false
```
Then start the planner just like above.

## Package structure

```text
nodes/
  mapper.py        — publishes occupancy grid
  path_planner.py  — RIG-tree receding horizon planner
  path_follower.py — follows waypoint paths
  position_controller.py
  yaw_controller.py
  rig_tree.py      — RIG-tree algorithm (Hollinger & Sukhatme 2014)
launch/
  simulation.launch.py  — Gazebo + IPP nodes (sim only)
  ipp.launch.py         — IPP nodes only (sim + real robot)
config/
  mapping_params.yaml   — tank dimensions, grid resolution
```
