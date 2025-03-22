cmake_minimum_required(VERSION 3.0.2)
project(state_observation_ros)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(state-observation REQUIRED)

## Declare catkin package
catkin_package()

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${state-observation_INCLUDE_DIRS}  # If your state-estimation library has its own include paths
)

## Macro to add a ROS observer node
macro(add_ros_observer observer_name)
    add_executable(${observer_name} src/${observer_name}.cpp)
    target_link_libraries(${observer_name} PRIVATE ${catkin_LIBRARIES} state-observation::state-observation)
    add_dependencies(${observer_name} ${catkin_EXPORTED_TARGETS})
endmacro()

## Add the ROS observer node
add_ros_observer(tilt_observer_ros)
