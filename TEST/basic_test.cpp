#include <gtest/gtest.h>
#include "LegsRobot.hpp"
#include <iostream>

TEST(constructor, pose)
{
  Pose<double> p;
  std::cout << "pose innitialze to: " << std::endl << p;
  EXPECT_TRUE(p.position().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(p.rotation().isApprox(Eigen::Matrix3d::Identity()));
}