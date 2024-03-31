#include <gtest/gtest.h>
#include "LegsRobot.hpp"
#include <iostream>


TEST(kinematics_test,joint_kinematics)
{
  Pose<double> leg_center;
  Joint<double> j1(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(), 0, leg_center);
  Joint<double> j2(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), 0, j1);

  Joint<double> j3 = j1;
  Joint<double> j4 = j2;
  
  j1.Forward_kinematics(M_PI / 3, leg_center);
  j2.Forward_kinematics(M_PI / 2, j1);

  std::cout << "joint 1:" << std::endl << j1 << std::endl;
  std::cout << "joint 2:" << std::endl << j2 << std::endl;

  j3.Inverse_kinematics(j1.rotation(), leg_center);
  j4.Inverse_kinematics(j2.rotation(), j1);

  EXPECT_EQ(j1, j3);
  EXPECT_EQ(j2, j4);
}
