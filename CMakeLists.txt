cmake_minimum_required(VERSION 3.5)
project(state_observation_ros)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(state-observation REQUIRED)

# Message files
set(msg_files
  "msg/state_msgs.msg"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
)

# Include directories
include_directories(
  include
  ${state-observation_INCLUDE_DIRS}
)


macro(add_ros_observer observer_name)
    add_executable(${observer_name} src/${observer_name}.cpp)
    target_link_libraries(${observer_name}
      PRIVATE
        state-observation::state-observation
    )
    ament_target_dependencies(${observer_name}
      rclcpp
      std_msgs
      sensor_msgs
      geometry_msgs
      message_filters
    )
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    add_dependencies(${observer_name} ${cpp_typesupport_target})
    install(TARGETS ${observer_name}
      DESTINATION lib/${PROJECT_NAME})
endmacro()

# Add the observer nodes
add_ros_observer(tilt_observer_ros)
add_ros_observer(tilt_observer_humanoid_ros)

# Install message headers
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

# ament package
ament_package()