#include <gtest/gtest.h>
#include <smaldog/kinematics/leg_ik_solver.h>

using namespace smaldog;

TEST(smaldog_ik_test, testLF)
{
  LegIKSolver<true, false> ik("lf", 0.050, 0.065, 0.088);
  EXPECT_EQ("lf", ik.getName());

  double p, f, k;

  /* Test default stance */
  bool ret = ik.solveIK(0.01, 0.06-0.019, -0.08, p, f, k);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(-0.11621866456, p, 0.000001);
  EXPECT_NEAR(1.215250663942, f, 0.000001);
  EXPECT_NEAR(-2.15274196197, k, 0.000001);

  /* Test out of range on X */
  ret = ik.solveIK(0.20, 0.06-0.019, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Y */
  ret = ik.solveIK(0.01, 0.2, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Z */
  ret = ik.solveIK(0.01, 0.06-0.019, -0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(0.01, 0.06-0.019, 0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(0.01, 0.06-0.019, 0.0, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(0.01, 0.06-0.019, -0.01, p, f, k);
  EXPECT_FALSE(ret);
}

TEST(smaldog_ik_test, testRF)
{
  LegIKSolver<true, true> ik("rf", 0.050, 0.065, 0.088);
  EXPECT_EQ("rf", ik.getName());

  double p, f, k;

  /* Test default stance */
  bool ret = ik.solveIK(0.01, -0.06+0.019, -0.08, p, f, k);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(0.11621866456, p, 0.000001);
  EXPECT_NEAR(1.215250663942, f, 0.000001);
  EXPECT_NEAR(-2.15274196197, k, 0.000001);

  /* Test out of range on X */
  ret = ik.solveIK(0.20, -0.06+0.019, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Y */
  ret = ik.solveIK(0.01, -0.2, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Z */
  ret = ik.solveIK(0.01, -0.06+0.019, -0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(0.01, -0.06+0.019, 0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(0.01, -0.06+0.019, 0.0, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(0.01, -0.06+0.019, -0.01, p, f, k);
  EXPECT_FALSE(ret);
}

TEST(smaldog_ik_test, testRR)
{
  LegIKSolver<false, true> ik("rr", 0.050, 0.065, 0.088);
  EXPECT_EQ("rr", ik.getName());

  double p, f, k;

  /* Test default stance */
  bool ret = ik.solveIK(-0.01, -0.06+0.019, -0.08, p, f, k);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(0.11621866456, p, 0.000001);
  EXPECT_NEAR(-1.215250663942, f, 0.000001);
  EXPECT_NEAR(2.15274196197, k, 0.000001);

  /* Test out of range on X */
  ret = ik.solveIK(-0.20, -0.06+0.019, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Y */
  ret = ik.solveIK(-0.01, -0.2, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Z */
  ret = ik.solveIK(-0.01, -0.06+0.019, -0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(-0.01, -0.06+0.019, 0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(-0.01, -0.06+0.019, 0.0, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(-0.01, -0.06+0.019, -0.01, p, f, k);
  EXPECT_FALSE(ret);
}

TEST(smaldog_ik_test, testLR)
{
  LegIKSolver<false, false> ik("lr", 0.050, 0.065, 0.088);
  EXPECT_EQ("lr", ik.getName());

  double p, f, k;

  /* Test default stance */
  bool ret = ik.solveIK(-0.01, 0.06-0.019, -0.08, p, f, k);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(-0.11621866456, p, 0.000001);
  EXPECT_NEAR(-1.215250663942, f, 0.000001);
  EXPECT_NEAR(2.15274196197, k, 0.000001);

  /* Test out of range on X */
  ret = ik.solveIK(-0.20, 0.06-0.019, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Y */
  ret = ik.solveIK(-0.01, 0.2, -0.08, p, f, k);
  EXPECT_FALSE(ret);

  /* Test out of range on Z */
  ret = ik.solveIK(-0.01, 0.06-0.019, -0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(-0.01, 0.06-0.019, 0.2, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(-0.01, 0.06-0.019, 0.0, p, f, k);
  EXPECT_FALSE(ret);
  ret = ik.solveIK(-0.01, 0.06-0.019, -0.01, p, f, k);
  EXPECT_FALSE(ret);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
