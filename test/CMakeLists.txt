catkin_add_nosetests(python)

catkin_add_gtest(test_leg_ik_solver test_leg_ik_solver.cpp)
target_link_libraries(test_leg_ik_solver
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_leg_fk_solver test_leg_fk_solver.cpp)
target_link_libraries(test_leg_fk_solver
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_kinematics_solver test_kinematics_solver.cpp)
target_link_libraries(test_kinematics_solver
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
