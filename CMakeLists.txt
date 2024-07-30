cmake_minimum_required(VERSION 3.5)
project(smaldog)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

find_package(Boost REQUIRED system thread)
find_package(orocos_kdl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

## Driver for communicating to hardware
add_executable(smaldog_driver
  src/driver.cpp
  src/trajectory_sampler.cpp
)
ament_target_dependencies(smaldog_driver
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2_ros
)
target_link_libraries(smaldog_driver
  ${Boost_LIBRARIES}
  "${cpp_typesupport_target}"
)
set_target_properties(smaldog_driver
  PROPERTIES OUTPUT_NAME driver
  PREFIX ""
)

## Walk controller
add_executable(smaldog_walk_controller
  src/walk_controller.cpp
)
ament_target_dependencies(smaldog_walk_controller
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2_kdl
  tf2_ros
)
target_link_libraries(smaldog_walk_controller
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
set_target_properties(smaldog_walk_controller
  PROPERTIES OUTPUT_NAME walk_controller
  PREFIX ""
)

## Loopback for debugging
add_executable(smaldog_loopback
  src/loopback.cpp
)
target_link_libraries(smaldog_loopback
  ${Boost_LIBRARIES}
)
set_target_properties(smaldog_loopback
  PROPERTIES OUTPUT_NAME loopback
  PREFIX ""
)

add_subdirectory(test)

install(
  TARGETS
    smaldog_driver
    smaldog_walk_controller
    smaldog_loopback
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
