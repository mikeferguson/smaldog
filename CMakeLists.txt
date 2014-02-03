cmake_minimum_required(VERSION 2.8.3)
project(smaldog)

find_package(Boost REQUIRED system thread)
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
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

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE__DIRS})

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

catkin_add_nosetests(tests)

install(
  TARGETS
    smaldog_loopback
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
