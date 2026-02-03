cmake_minimum_required(VERSION 3.0.2)
project(sp4_panda_ctrl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    moveit_ros_planning_interface
    moveit_core
    moveit_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Executable: panda_cli_control (now split into multiple source files)
add_executable(panda_cli_control
  src/main.cpp
  src/panda_controller.cpp
  src/cli_interface.cpp
  src/sp_interface.cpp
)

target_link_libraries(panda_cli_control
  ${catkin_LIBRARIES}
)

# Executable: coordinate_translator (unchanged)
add_executable(coordinate_translator
  src/coordinate_translator.cpp
)

target_link_libraries(coordinate_translator
  ${catkin_LIBRARIES}
)
