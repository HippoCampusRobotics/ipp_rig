cmake_minimum_required(VERSION 3.5)
project(ipp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

############################################################################
# MESSAGES
############################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PolygonsStamped.msg
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

############################################################################
# OBSTACLE PUBLISHER NODE
############################################################################
add_executable(obstacle_publisher
  src/obstacle_publisher/obstacle_publisher_node.cpp
)
target_include_directories(obstacle_publisher PRIVATE src/obstacle_publisher)
ament_target_dependencies(obstacle_publisher rclcpp geometry_msgs ament_index_cpp)
target_link_libraries(obstacle_publisher
  "${cpp_typesupport_target}"
  yaml-cpp
)

############################################################################
# INSTALL
############################################################################
install(TARGETS obstacle_publisher
  DESTINATION lib/${PROJECT_NAME}
)

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
