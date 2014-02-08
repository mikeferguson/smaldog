cmake_minimum_required(VERSION 2.8.3)
project(smaldog)

find_package(Boost REQUIRED system thread)
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    control_msgs
    message_generation
    sensor_msgs
    tf
    tf_conversions
    roscpp
)

add_message_files(
  FILES
    State.msg
)

catkin_python_setup()

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    roscpp
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# this is a hack, will eventually be unneeded once orocos-kdl is fixed
link_directories(${orocos_kdl_LIBRARY_DIRS})

## Driver for communicating to hardware
add_executable(smaldog_driver
  src/driver.cpp
  src/trajectory_sampler.cpp
)
target_link_libraries(smaldog_driver
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
set_target_properties(smaldog_driver
  PROPERTIES OUTPUT_NAME driver
  PREFIX ""
)
add_dependencies(smaldog_driver smaldog_gencpp)

## Walk controller
add_executable(smaldog_walk_controller
  src/walk_controller.cpp
)
target_link_libraries(smaldog_walk_controller
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
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
  ${catkin_LIBRARIES}
)
set_target_properties(smaldog_loopback
  PROPERTIES OUTPUT_NAME loopback
  PREFIX ""
)

install(
  TARGETS
    smaldog_driver
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(test)
