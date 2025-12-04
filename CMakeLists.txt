cmake_minimum_required(VERSION 3.0.2)
project(sp3_panda_ctrl)

## Enable C++11
add_compile_options(-std=c++11)

## Find required ROS package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)

## Configure Catkin Packages
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    moveit_ros_planning_interface
    moveit_core
    moveit_msgs
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Executable: panda_cli_control
add_executable(panda_cli_control
  src/panda_cli_control.cpp
)

target_link_libraries(panda_cli_control
  ${catkin_LIBRARIES}
)

## Executable: sp_test_node
add_executable(coordinate_translator
  src/coordinate_translator.cpp
)

target_link_libraries(coordinate_translator
  ${catkin_LIBRARIES}
)
