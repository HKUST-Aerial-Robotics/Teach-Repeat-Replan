#include <gtest/gtest.h>

#include <algorithm>
#include <ctime>

#include "translation_control.hpp"

TEST(language_test, init_test)
{
  mocka::TranslationController ctrl;

  EXPECT_NE(reinterpret_cast<uint64_t>(ctrl.getXController()), 0);
  EXPECT_NE(reinterpret_cast<uint64_t>(ctrl.getYController()), 0);
  EXPECT_NE(reinterpret_cast<uint64_t>(ctrl.getZController()), 0);
}

TEST(language_test, coverage_setFreq_test)
{
  mocka::TranslationController ctrl;
  ctrl.setFreq(10);
  EXPECT_EQ(ctrl.getXController()->getFreq(), 10);
  EXPECT_EQ(ctrl.getYController()->getFreq(), 10);
  EXPECT_EQ(ctrl.getZController()->getFreq(), 10);
  EXPECT_EQ(ctrl.getFreq(), 10);
}

TEST(language_test, coeff_setting)
{
  mocka::TranslationController ctrl;
  Eigen::Matrix3d              coeff;
  coeff << 3, 4, 5, 6, 7, 8, 9, 10, 11;
  ctrl.setCoeff(coeff);
  EXPECT_EQ(ctrl.getXController()->getKp(), 3);
  EXPECT_EQ(ctrl.getXController()->getKi(), 4);
  EXPECT_EQ(ctrl.getXController()->getKd(), 5);
  EXPECT_EQ(ctrl.getYController()->getKp(), 6);
  EXPECT_EQ(ctrl.getYController()->getKi(), 7);
  EXPECT_EQ(ctrl.getYController()->getKd(), 8);
  EXPECT_EQ(ctrl.getZController()->getKp(), 9);
  EXPECT_EQ(ctrl.getZController()->getKi(), 10);
  EXPECT_EQ(ctrl.getZController()->getKd(), 11);
}

TEST(pid_test, init_test)
{
  mocka::TranslationController ctrl;

  mocka::PID* x;

  for (int i = 0; i < 3; ++i)
  {
    switch (i)
    {
      case 0:
        x = ctrl.getXController();
        break;
      case 1:
        x = ctrl.getYController();
        break;
      case 2:
        x = ctrl.getZController();
        break;
    }
    EXPECT_EQ(x->getKp(), 1);
    EXPECT_EQ(x->getKi(), 0);
    EXPECT_EQ(x->getKd(), 0);
  }
}

TEST(pid_test, spin_test)
{
  mocka::PID ctrl;
  ctrl.setFeeds(0);
  ctrl.setInput(1);
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput(), 1);
}

TEST(pid_test, derivative_system_test)
{
  std::cout << std::setprecision(2) << std::endl;
  mocka::PID ctrl;
  double     totalE          = 0;
  double     error           = 0;
  double     groundTruth     = 0;
  double     groundTruth_dot = 0;
  for (int i = 0; i < 1000; ++i)
  {
    groundTruth_dot = ctrl.getOutput();
    groundTruth += groundTruth_dot * 0.01;
    std::cout << "\t " << groundTruth;
    if (i % 10 == 9)
      std::cout << std::endl;
    ctrl.setFeeds(groundTruth);
    ctrl.setInput(1);
    error = groundTruth - 1;
    ctrl.spinOnce();
    totalE += fabs(error);
  }
  std::cout << std::endl << "errors" << error << " " << totalE << std::endl;
  EXPECT_LT(fabs(error), 0.01);
}

TEST(pid_test, derivative2_system_test)
{
  std::cout << std::setprecision(2) << std::endl;
  mocka::PID ctrl;
  ctrl.setKd(0.1);
  double totalE           = 0;
  double error            = 0;
  double groundTruth      = 0;
  double groundTruth_dot  = 0;
  double groundTruth_dot2 = 0;
  for (int i = 0; i < 1000; ++i)
  {
    groundTruth_dot2 = ctrl.getOutput();
    groundTruth_dot *= 0.99;
    groundTruth_dot += groundTruth_dot2 * 0.5;
    groundTruth += groundTruth_dot * 0.5;
    std::cout << "\t " << groundTruth;
    if (i % 10 == 9)
      std::cout << std::endl;
    ctrl.setFeeds(groundTruth);
    ctrl.setInput(1);
    error = groundTruth - 1;
    ctrl.spinOnce();
    totalE += fabs(error);
  }
  std::cout << std::endl
            << "errors" << fabs(error) << " " << totalE << std::endl;

  EXPECT_LT(fabs(error), 0.01);
}

TEST(pid_test, intergration_system_test)
{
  std::cout << std::setprecision(2) << std::endl;
  mocka::PID ctrl;
  ctrl.setKd(0.1);
  ctrl.setKi(0.1);
  double totalE           = 0;
  double error            = 0;
  double groundTruth      = 0;
  double groundTruth_dot  = 0;
  double groundTruth_dot2 = 0;
  for (int i = 0; i < 1000; ++i)
  {
    groundTruth_dot2 = ctrl.getOutput();
    groundTruth_dot *= 0.9;
    groundTruth_dot += groundTruth_dot2 * 0.5;
    groundTruth *= 0.98;
    groundTruth += groundTruth_dot * 0.5;
    std::cout << "\t| " << groundTruth << "\t" << ctrl.getIntergrade();
    if (i % 5 == 4)
      std::cout << std::endl;
    ctrl.setFeeds(groundTruth);
    ctrl.setInput(1);
    error = groundTruth - 1;
    ctrl.spinOnce();
    totalE += fabs(error);
  }
  std::cout << std::endl
            << "errors" << fabs(error) << " " << totalE << std::endl;

  EXPECT_LT(fabs(error), 0.01);
}

TEST(pid_test, noisey_system_test)
{
  std::srand(std::time(0)); // use current time as seed for random generator
  std::cout << std::setprecision(2) << std::endl;
  mocka::PID ctrl;
  ctrl.setKd(0.1);
  ctrl.setKi(0.1);
  double totalE           = 0;
  double error            = 0;
  double groundTruth      = 0;
  double groundTruth_dot  = 0;
  double groundTruth_dot2 = 0;
  for (int i = 0; i < 1000; ++i)
  {
    groundTruth_dot2 = ctrl.getOutput();
    groundTruth_dot *= 0.9;
    groundTruth_dot += groundTruth_dot2 * 0.5;
    groundTruth *= 0.98;
    groundTruth += groundTruth_dot * 0.5;

    std::cout << "\t| " << groundTruth << "\t" << ctrl.getIntergrade();
    if (i % 5 == 4)
      std::cout << std::endl;
    ctrl.setFeeds(groundTruth + 0.00001 * (std::rand() % 1000) - 0.005);
    ctrl.setInput(1);
    error = groundTruth - 1;
    ctrl.spinOnce();
    totalE += fabs(error);
  }
  std::cout << std::endl
            << "errors" << fabs(error) << " " << totalE << std::endl;

  EXPECT_LT(fabs(error), 0.03); //! @ %1 error + noise base
}

TEST(pid_test, intergration2_system_test)
{
  std::cout << std::setprecision(2) << std::endl;
  mocka::PID ctrl;
  ctrl.setKd(0.05);
  ctrl.setKi(0.01);
  double totalE           = 0;
  double error            = 0;
  double groundTruth      = 0;
  double groundTruth_dot  = 0;
  double groundTruth_dot2 = 0;
  for (int i = 0; i < 300; ++i)
  {
    groundTruth_dot2 = ctrl.getOutput();
    groundTruth_dot *= 0.9;
    groundTruth_dot += groundTruth_dot2 * 0.9;
    groundTruth += groundTruth_dot * 0.5;
    groundTruth -= 0.5;
    std::cout << "\t| " << groundTruth << "\t" << ctrl.getIntergrade();
    if (i % 5 == 4)
      std::cout << std::endl;
    ctrl.setFeeds(groundTruth);
    ctrl.setInput(1);
    error = groundTruth - 1;
    ctrl.spinOnce();
    totalE += fabs(error);
  }
  std::cout << std::endl
            << "errors" << fabs(error) << " " << totalE << std::endl;

  EXPECT_LT(fabs(error), 0.01);
}

TEST(translation_test, spin_test)
{
  mocka::TranslationController ctrl;
  ctrl.setFeeds(Eigen::Vector3d(0, 0, 0));
  ctrl.setInput(Eigen::Vector3d(1, 1, 1));
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput(), Eigen::Vector3d(1, 1, 1));
}

TEST(translation_test, derivative_system_test)
{
  std::cout << std::setprecision(2) << std::endl;
  mocka::TranslationController ctrl;

  Eigen::Vector3d totalE(0, 0, 0);
  Eigen::Vector3d error(0, 0, 0);
  Eigen::Vector3d groundTruth(0, 0, 0);
  Eigen::Vector3d groundTruth_dot(0, 0, 0);

  Eigen::MatrixXd print(3, 10);
  for (int i = 0; i < 1000; ++i)
  {
    groundTruth_dot = ctrl.getOutput();
    groundTruth += groundTruth_dot * 0.01;
    print.col(i % 10) = groundTruth;
    if (i % 10 == 9)
      std::cout << print << std::endl << std::endl;
    ctrl.setFeeds(groundTruth);
    ctrl.setInput(Eigen::Vector3d(1, 1, 1));
    error = groundTruth - Eigen::Vector3d(1, 1, 1);
    ctrl.spinOnce();
    totalE += error.array().abs().matrix();
  }
  std::cout << std::endl << "errors" << error << " " << totalE << std::endl;
  EXPECT_LT(error.array().abs().matrix().norm(),
            Eigen::Vector3d(0.01, 0.01, 0.01).norm());
}

TEST(translation_test, intergration2_system_test)
{
  std::cout << std::setprecision(2) << std::endl;
  mocka::TranslationController ctrl;

  ctrl.getXController()->setKd(0.05);
  ctrl.getYController()->setKd(0.05);
  ctrl.getZController()->setKd(0.05);

  ctrl.getXController()->setKi(0.01);
  ctrl.getYController()->setKi(0.01);
  ctrl.getZController()->setKi(0.01);

  Eigen::Vector3d totalE(0, 0, 0);
  Eigen::Vector3d error(0, 0, 0);
  Eigen::Vector3d groundTruth(0, 0, 0);
  Eigen::Vector3d groundTruth_dot(0, 0, 0);
  Eigen::Vector3d groundTruth_dot2(0, 0, 0);

  Eigen::MatrixXd print(3, 10);
  for (int i = 0; i < 300; ++i)
  {

    groundTruth_dot2 = ctrl.getOutput();
    groundTruth_dot *= 0.9;
    groundTruth_dot += groundTruth_dot2 * 0.9;
    groundTruth += groundTruth_dot * 0.5;
    groundTruth -= Eigen::Vector3d(0.5, 0.5, 0.5);

    print.col(i % 10) = groundTruth;
    if (i % 10 == 9)
      std::cout << print << std::endl << std::endl;
    ctrl.setFeeds(groundTruth);
    ctrl.setInput(Eigen::Vector3d(1, 1, 1));
    error = groundTruth - Eigen::Vector3d(1, 1, 1);
    ctrl.spinOnce();
    totalE += error.array().abs().matrix();
  }
  std::cout << std::endl << "errors" << error << " " << totalE << std::endl;
  EXPECT_LT(error.array().abs().matrix().norm(),
            Eigen::Vector3d(0.01, 0.01, 0.01).norm());
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
