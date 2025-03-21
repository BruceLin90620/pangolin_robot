cmake_minimum_required(VERSION 3.5)

project(routing_agent_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs)

set (INTERFACES_LIST
  "msg/StampedArray4m.msg"
  "msg/StampedBoolean.msg"
  "msg/StampedString.msg"
  "srv/ORAgentMsg.srv"
  "srv/RoutingServiceMsg.srv"
  "srv/NavServiceMsg.srv"
  "srv/MergeWaypointGraphServiceMsg.srv"
  "srv/LoadWaypointGraphServiceMsg.srv"
)

set (DEPENDENCIES_LIST
  "std_msgs"
)


## build actions only if using ROS2 crystal and action_msgs_FOUND
if (action_msgs_FOUND)
  set (INTERFACES_LIST ${INTERFACES_LIST} "action/Fibonacci.action")
  set (DEPENDENCIES_LIST ${DEPENDENCIES_LIST} "action_msgs")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACES_LIST}
  DEPENDENCIES ${DEPENDENCIES_LIST}
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
