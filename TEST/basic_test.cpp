#include <gtest/gtest.h>
#include "LegsRobot.hpp"
#include <iostream>

TEST(constructor, robot)
{
  Pose<double> p;
  std::cout << "pose innitialze to: " << std::endl << p;
  EXPECT_TRUE(p.position().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(p.rotation().isApprox(Eigen::Matrix3d::Identity()));

  //test copy constructor
  Pose<double> cpy_p(p);
  EXPECT_EQ(p, cpy_p);

  //test assign operator
  cpy_p = p;
  EXPECT_EQ(p, cpy_p);

  Joint<double> j(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0, p);
  EXPECT_TRUE(j.position().isApprox(p.position()));
  EXPECT_TRUE(j.rotation().isApprox(Eigen::Matrix3d::Identity()));

  //using no zero relative position and angle
  Joint<double> j2(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ(), M_PI / 3, p);
  std::cout << "joint innitialze to: " << std::endl << j2;
  Eigen::Matrix3d ref_rotation{
    {0.500000000000000,-0.866025403784439,0},
    {0.866025403784439,0.500000000000000,0},
    {0, 0,1.000000000000000}
  };
  EXPECT_TRUE(j2.rotation().isApprox(ref_rotation));

  //test joint copy constructor
  Joint<double> j3(j2);
  EXPECT_EQ(j2, j3);

  //tedt assignment operator
  j3 = j2;
  EXPECT_EQ(j2, j3);

  //test legs robot constructor
  LegsRobot<double> legsrobot;
  std::cout << "legs robot initialise to: " << std::endl << legsrobot;
}