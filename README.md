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

# Terminal 2: start the planner — runs continuously until stopped
ros2 service call /bluerov00/path_planner/start std_srvs/srv/Trigger {}

# Terminal 2: stop the planner
ros2 service call /bluerov00/path_planner/stop std_srvs/srv/Trigger {}
# Note: start/stop can be called repeatedly to pause and resume the mission.
```

## Run (real robot)

```bash
ros2 launch ipp ipp.launch.py vehicle_name:=bluerov00 use_sim_time:=false
```
Then start the planner just like above.

## Package structure

```text
nodes/
  rig_tree.py      — RIG-tree algorithm (Hollinger & Sukhatme 2014, Alg. 2)
  path_planner.py  — receding-horizon planning loop (RIG-tree → path_follower)
  path_follower.py — pure-pursuit waypoint follower
  mapper.py        — publishes occupancy grid from obstacle data
  position_controller.py — P-control in x/y/z
  yaw_controller.py      — P-control for yaw
launch/
  simulation.launch.py  — Gazebo + IPP nodes (sim only)
  ipp.launch.py         — IPP nodes only (sim + real robot)
config/
  mapping_params.yaml   — tank dimensions, grid resolution
NOTE.md                 — plain-language system explanation (git-ignored)
```
