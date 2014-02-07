#include <gtest/gtest.h>
#include <smaldog/kinematics/leg_fk_solver.h>

using namespace smaldog;

TEST(smaldog_fk_test, testLF)
{
  LegFKSolver<true, false> fk("lf", 0.050, 0.065, 0.088);
  EXPECT_EQ("lf", fk.getName());

  double x, y, z;

  /* Test default stance */
  bool ret = fk.solveFK(-0.11621866456, 1.215250663942, -2.15274196197, x, y, z);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(0.01, x, 0.000001);
  EXPECT_NEAR(0.06-0.019, y, 0.000001);
  EXPECT_NEAR(-0.08, z, 0.000001);
}

TEST(smaldog_fk_test, testRF)
{
  LegFKSolver<true, true> fk("rf", 0.050, 0.065, 0.088);
  EXPECT_EQ("rf", fk.getName());

  double x, y, z;

  /* Test default stance */
  bool ret = fk.solveFK(0.11621866456, 1.215250663942, -2.15274196197, x, y, z);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(0.01, x, 0.000001);
  EXPECT_NEAR(-0.06+0.019, y, 0.000001);
  EXPECT_NEAR(-0.08, z, 0.000001);
}

TEST(smaldog_fk_test, testRR)
{
  LegFKSolver<false, true> fk("rr", 0.050, 0.065, 0.088);
  EXPECT_EQ("rr", fk.getName());

  double x, y, z;

  /* Test default stance */
  bool ret = fk.solveFK(0.11621866456, -1.215250663942, 2.15274196197, x, y, z);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(-0.01, x, 0.000001);
  EXPECT_NEAR(-0.06+0.019, y, 0.000001);
  EXPECT_NEAR(-0.08, z, 0.000001);
}

TEST(smaldog_fk_test, testLR)
{
  LegFKSolver<false, false> fk("lr", 0.050, 0.065, 0.088);
  EXPECT_EQ("lr", fk.getName());

  double x, y, z;

  /* Test default stance */
  bool ret = fk.solveFK(-0.11621866456, -1.215250663942, 2.15274196197, x, y, z);
  EXPECT_TRUE(ret);
  EXPECT_NEAR(-0.01, x, 0.000001);
  EXPECT_NEAR(0.06-0.019, y, 0.000001);
  EXPECT_NEAR(-0.08, z, 0.000001);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
