cmake_minimum_required(VERSION 3.0.2)
project(state_observation_ros)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  message_filters 
)

find_package(state-observation REQUIRED)



add_message_files(
  FILES
  state_msgs.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs message_filters message_runtime 
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${state-observation_INCLUDE_DIRS} 
)

## Macro to add a ROS observer node
macro(add_ros_observer observer_name)
    add_executable(${observer_name} src/${observer_name}.cpp)
    target_link_libraries(${observer_name} PRIVATE ${catkin_LIBRARIES} state-observation::state-observation)
    add_dependencies(${observer_name} ${catkin_EXPORTED_TARGETS})
endmacro()

## Add the ROS observer node
add_ros_observer(tilt_observer_ros)
add_ros_observer(tilt_observer_humanoid_ros)