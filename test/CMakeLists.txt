#catkin_add_nosetests(python)

ament_add_gtest(test_leg_ik_solver test_leg_ik_solver.cpp)
target_link_libraries(test_leg_ik_solver
  ${Boost_LIBRARIES}
)

ament_add_gtest(test_leg_fk_solver test_leg_fk_solver.cpp)
target_link_libraries(test_leg_fk_solver
  ${Boost_LIBRARIES}
)

ament_add_gtest(test_kinematics_solver test_kinematics_solver.cpp)
target_link_libraries(test_kinematics_solver
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)
