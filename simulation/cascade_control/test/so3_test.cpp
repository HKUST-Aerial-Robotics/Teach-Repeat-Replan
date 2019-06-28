#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "control_states.hpp"

TEST(so3test, SO3_init_test)
{
  mocka::SO3      so3;
  Eigen::Matrix3d test;
  test << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  EXPECT_EQ(so3.quaternion.toRotationMatrix(), test);
  EXPECT_DOUBLE_EQ(so3.thrust, 9.81);
}

TEST(se3test, SE3_init_test)
{
  mocka::SE3      se3;
  Eigen::Vector3d test(0, 0, 9.81);
  Eigen::Vector3d testVP(1, 0, 0);
  EXPECT_EQ(se3.acceleration, test);
  EXPECT_EQ(se3.viewPoint, testVP);
}

TEST(so3se3, SO3_to_SE3_normal)
{
  mocka::SO3 so3;
  mocka::SE3 ans =
    so3.toSE3(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1));
  mocka::SE3 test;
  EXPECT_EQ(ans.acceleration, test.acceleration);
  EXPECT_EQ(ans.viewPoint, test.viewPoint);
}

TEST(so3se3, SE3_to_SO3_normal)
{
  mocka::SE3 se3;
  mocka::SO3 ans =
    se3.toSO3(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1));
  mocka::SO3 test;
  EXPECT_EQ(ans.thrust, test.thrust);
  EXPECT_EQ(ans.quaternion.toRotationMatrix(),
            test.quaternion.toRotationMatrix());
  //  debug info
  //  std::cout << ans.quaternion.toRotationMatrix() << std::endl
  //            << test.quaternion.toRotationMatrix();
}

TEST(so3se3, SE3_to_SO3_View45deg)
{
  mocka::SE3 se3;
  mocka::SO3 ans =
    se3.toSO3(Eigen::Vector3d(sqrt(2), sqrt(2), 0), Eigen::Vector3d(0, 0, 1));
  mocka::SO3 test;
  test.quaternion = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d(0, 0, -1)));
  test.quaternion.normalize();
  EXPECT_EQ(ans.thrust, test.thrust);
  EXPECT_DOUBLE_EQ(
    (ans.quaternion.toRotationMatrix() - test.quaternion.toRotationMatrix())
      .norm(),
    0);
  //  debug info
  //  std::cout << ans.quaternion.toRotationMatrix() << std::endl
  //            << test.quaternion.toRotationMatrix();
}

TEST(so3se3, SE3_to_SO3_ViewN45deg)
{
  mocka::SE3 se3;
  mocka::SO3 ans =
    se3.toSO3(Eigen::Vector3d(-sqrt(2), -sqrt(2), 0), Eigen::Vector3d(0, 0, 1));
  mocka::SO3 test;
  test.quaternion = Eigen::Quaterniond(
    Eigen::AngleAxisd(1.25 * M_PI, Eigen::Vector3d(0, 0, -1)));
  test.quaternion.normalize();
  EXPECT_EQ(ans.thrust, test.thrust);
  EXPECT_NEAR(
    (ans.quaternion.toRotationMatrix() - test.quaternion.toRotationMatrix())
      .norm(),
    0, 1e-10);
  //  debug info
  //  std::cout << ans.quaternion.toRotationMatrix() << std::endl
  //            << test.quaternion.toRotationMatrix();
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
