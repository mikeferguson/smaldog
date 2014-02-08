#include <gtest/gtest.h>
#include <smaldog/kinematics/kinematics_solver.h>

using namespace smaldog;

TEST(smaldog_kinematics_test, ik_test_default_stance)
{
  KinematicsSolver k(0.038, 0.172, 0.050, 0.065, 0.088);

  KDL::Vector lf(0.096, 0.06, -0.08);
  KDL::Vector rr(-0.096, -0.06, -0.08);
  KDL::Vector rf(0.096, -0.06, -0.08);
  KDL::Vector lr(-0.096, 0.06, -0.08);

  RobotState state;
  state.odom_transform.p.z(0.08);

  EXPECT_TRUE(k.solveIK(lf, rr, rf, lr, 0.0, 0.0, 0.0, state));

  // lf leg joints
  EXPECT_NEAR(-0.11621866456, state.joint_positions[1], 0.000001);
  EXPECT_NEAR(1.215250663942, state.joint_positions[3], 0.000001);
  EXPECT_NEAR(-2.15274196197, state.joint_positions[5], 0.000001);

  // rr leg joints
  EXPECT_NEAR(0.11621866456, state.joint_positions[6], 0.000001);
  EXPECT_NEAR(-1.215250663942, state.joint_positions[8], 0.000001);
  EXPECT_NEAR(2.15274196197, state.joint_positions[10], 0.000001);

  // rf leg joints
  EXPECT_NEAR(0.11621866456, state.joint_positions[0], 0.000001);
  EXPECT_NEAR(1.215250663942, state.joint_positions[2], 0.000001);
  EXPECT_NEAR(-2.15274196197, state.joint_positions[4], 0.000001);

  // lr leg joints
  EXPECT_NEAR(-0.11621866456, state.joint_positions[7], 0.000001);
  EXPECT_NEAR(-1.215250663942, state.joint_positions[9], 0.000001);
  EXPECT_NEAR(2.15274196197, state.joint_positions[11], 0.000001);

  double x, y, z, w;
  state.odom_transform.M.GetQuaternion(x, y, z, w);

  EXPECT_NEAR(0.0, x, 0.000001);
  EXPECT_NEAR(0.0, y, 0.000001);
  EXPECT_NEAR(0.0, z, 0.000001);
  EXPECT_NEAR(1.0, w, 0.000001);

  EXPECT_NEAR(0.0, state.odom_transform.p.x(), 0.000001);
  EXPECT_NEAR(0.0, state.odom_transform.p.y(), 0.000001);
  EXPECT_NEAR(0.08, state.odom_transform.p.z(), 0.000001);
}

TEST(smaldog_kinematics_test, ik_test_pitch_stance)
{
  KinematicsSolver k(0.038, 0.172, 0.050, 0.065, 0.088);

  KDL::Vector lf(0.096, 0.06, -0.08);
  KDL::Vector rr(-0.096, -0.06, -0.08);
  KDL::Vector rf(0.096, -0.06, -0.08);
  KDL::Vector lr(-0.096, 0.06, -0.08);

  RobotState state;
  state.odom_transform.p.z(0.08);

  EXPECT_TRUE(k.solveIK(lf, rr, rf, lr, 0.0, 0.1, 0.0, state));

  // lf leg joints
  EXPECT_NEAR(-0.1313477432, state.joint_positions[1], 0.000001);
  EXPECT_NEAR(1.31145158059, state.joint_positions[3], 0.000001);
  EXPECT_NEAR(-2.2949729657, state.joint_positions[5], 0.000001);

  // rr leg joints
  EXPECT_NEAR(0.10430076344, state.joint_positions[6], 0.000001);
  EXPECT_NEAR(-1.1113089435, state.joint_positions[8], 0.000001);
  EXPECT_NEAR(2.00684394568, state.joint_positions[10], 0.000001);

  // rf leg joints
  EXPECT_NEAR(0.13134774328, state.joint_positions[0], 0.000001);
  EXPECT_NEAR(1.31145158059, state.joint_positions[2], 0.000001);
  EXPECT_NEAR(-2.2949729657, state.joint_positions[4], 0.000001);

  // lr leg joints
  EXPECT_NEAR(-0.1043007634, state.joint_positions[7], 0.000001);
  EXPECT_NEAR(-1.1113089435, state.joint_positions[9], 0.000001);
  EXPECT_NEAR(2.00684394568, state.joint_positions[11], 0.000001);
}

TEST(smaldog_kinematics_test, ik_test_out_of_bounds)
{
  KinematicsSolver k(0.038, 0.172, 0.050, 0.065, 0.088);

  KDL::Vector lf(0.096, 0.06, -0.08);
  KDL::Vector rr(-0.25, -0.06, -0.08);
  KDL::Vector rf(0.096, -0.06, -0.08);
  KDL::Vector lr(-0.096, 0.06, -0.08);

  RobotState state;

  EXPECT_FALSE(k.solveIK(lf, rr, rf, lr, 0.0, 0.0, 0.0, state));

  // lf leg joints -- make sure this computation went forward ok
  EXPECT_NEAR(-0.11621866456, state.joint_positions[1], 0.000001);
  EXPECT_NEAR(1.215250663942, state.joint_positions[3], 0.000001);
  EXPECT_NEAR(-2.15274196197, state.joint_positions[5], 0.000001);

  // rr leg joints - pitch should be OK
  EXPECT_NEAR(0.11621866456, state.joint_positions[6], 0.000001);
  // IK fails at this point, no need to test further
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
