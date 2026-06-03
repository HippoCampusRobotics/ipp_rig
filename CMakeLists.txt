cmake_minimum_required(VERSION 3.5)
project(ipp)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


install(PROGRAMS
  nodes/mapper.py
  nodes/path_follower.py
  nodes/path_planner.py
  nodes/position_controller.py
  nodes/yaw_controller.py
  nodes/rig_tree.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
