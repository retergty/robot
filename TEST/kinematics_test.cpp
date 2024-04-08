#include <gtest/gtest.h>
#include "LegsRobot.hpp"
#include <iostream>


template<typename scalar>
void LegRobotEquleT(const LegsRobot<scalar>& lhs, const LegsRobot<scalar>& rhs)
{
  auto right_l = lhs.rightLeg().cbegin();
  auto right_r = rhs.rightLeg().cbegin();
  auto left_l = lhs.leftLeg().cbegin();
  auto left_r = rhs.leftLeg().cbegin();

  for (;right_l != lhs.rightLeg().cend();++right_l, ++right_r)
  {
    EXPECT_EQ(*right_l, *right_r);
  }
  for (;left_l != lhs.leftLeg().cend();++left_l, ++left_r)
  {
    EXPECT_EQ(*left_l, *left_r);
  }
}

TEST(kinematics_test, joint_kinematics)
{
  Pose<double> leg_center;
  Joint<double> j1(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(), 0, leg_center);
  Joint<double> j2(Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), 0, j1);

  Joint<double> j3 = j1;
  Joint<double> j4 = j2;
  
  j1.Forward_kinematics(M_PI / 3, leg_center);
  j2.Forward_kinematics(M_PI / 2, j1);

  //std::cout << "joint 1:" << std::endl << j1 << std::endl;
  //std::cout << "joint 2:" << std::endl << j2 << std::endl;

  j3.Inverse_kinematics(j1.rotation(), leg_center);
  j4.Inverse_kinematics(j2.rotation(), j1);

  EXPECT_EQ(j1, j3);
  EXPECT_EQ(j2, j4);
}

TEST(kinematics_test, robot_kinematics)
{
  LegsRobot<double> rob1;
  LegsRobot<double> rob2;
  Eigen::Vector<double, 12> angle;
  angle.setZero();
  rob1.Forward_kinematics(angle);
  rob2.Inverse_kinematics(rob1.legCenter(), rob1.rightLeg().back(), rob1.leftLeg().back());
  EXPECT_EQ(rob1, rob2);

  for (size_t i = 0;i < 12;++i) {
    angle(i) = M_PI / 2 / (i + 1);
  }
  rob1.Forward_kinematics(angle);
  rob2.Inverse_kinematics(rob1.legCenter(), rob1.rightLeg().back(), rob1.leftLeg().back());
  LegRobotEquleT(rob1, rob2);

  EXPECT_EQ(rob1, rob2);
  
  for (size_t i = 0;i < 12;++i) {
    angle(i) = M_PI / (i + 1) - 0.1;
  }
  rob1.Forward_kinematics(angle);
  rob2.Inverse_kinematics(rob1.legCenter(), rob1.rightLeg().back(), rob1.leftLeg().back());
  EXPECT_EQ(rob1, rob2);
}
