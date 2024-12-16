cmake_minimum_required(VERSION 3.0.2)
project(potbot_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  geometry_msgs
  navfn
  angles
  base_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  sensor_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  potbot_lib
  potbot_msgs
  potbot_base
)

generate_dynamic_reconfigure_options(
  cfg/StatePlugin.cfg
  cfg/PID.cfg
  cfg/PurePursuit.cfg
  cfg/TimeState.cfg
  cfg/OptimalPathFollower.cfg
  cfg/TraditionalAPF.cfg
  cfg/PersonalSpaceDWA.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} potbot_lib test_pkg
  # CATKIN_DEPENDS
  #   costmap_2d
  #   dynamic_reconfigure
  #   geometry_msgs
  #   nav_core
  #   navfn
  #   nav_msgs
  #   pluginlib
  #   roscpp
  #   tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/potbot_plan.cpp
  src/local_planner.cpp
  src/state_layer.cpp
  src/pid.cpp
  src/pure_pursuit.cpp
  src/time_state.cpp
  src/optimal_path_follower.cpp
  src/traditional_apf.cpp
  src/personal_space_dwa.cpp
  src/personal_space_dwa_ros.cpp
  src/restricted_costmap.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-std=c++14)
