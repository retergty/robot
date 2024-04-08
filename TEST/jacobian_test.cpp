#include <gtest/gtest.h>
#include "LegsRobot.hpp"
#include <iostream>

TEST(jacobian_test, robot_jacobian_init)
{
  LegsRobot<double> rob1;
  rob1.Update_jacobian();
  std::cout << std::endl << rob1.JacobianRight() << std::endl;
  std::cout << std::endl << rob1.JacobianLeft() << std::endl;
  Eigen::Vector<double, 6> angle_rate;
  angle_rate.setZero();

  Eigen::Vector<double, 3> end_effctor_vel_right = rob1.End_effctor_vel<LegsRobot<double>::LEG::RIGHT>(Eigen::Vector3d::UnitX(), angle_rate);
  Eigen::Vector<double, 3> end_effctor_vel_left = rob1.End_effctor_vel<LegsRobot<double>::LEG::LEFT>(Eigen::Vector3d::UnitX(), angle_rate);
  Eigen::Vector<double, 3> end_effctor_vel_right_direct = rob1.End_effctor_vel_direct<LegsRobot<double>::LEG::RIGHT>(Eigen::Vector3d::UnitX(), angle_rate);
  Eigen::Vector<double, 3> end_effctor_vel_left_direct = rob1.End_effctor_vel_direct<LegsRobot<double>::LEG::LEFT>(Eigen::Vector3d::UnitX(), angle_rate);

  std::cout << std::endl
    << end_effctor_vel_right
    << std::endl;
  std::cout << std::endl <<
    end_effctor_vel_left
    << std::endl;
  std::cout << std::endl
    << end_effctor_vel_right_direct
    << std::endl;
  std::cout << std::endl
    << end_effctor_vel_left_direct
    << std::endl;

  EXPECT_TRUE(end_effctor_vel_right.isApprox(end_effctor_vel_right_direct, 1e-6));
  EXPECT_TRUE(end_effctor_vel_left.isApprox(end_effctor_vel_left_direct, 1e-6));
}

TEST(jacobian_test, robot_jacobian_normal)
{
  LegsRobot<double> rob1;
  rob1.Inverse_kinematics(Pose<double>({ 0,0,-0.08 }), rob1.rightLeg().back(), rob1.leftLeg().back());
  std::cout << std::endl << rob1.leftLegAngle() << std::endl;
  std::cout << std::endl << rob1.rightLegAngle() << std::endl;

  rob1.Update_jacobian();
  std::cout << std::endl << rob1.JacobianRight() << std::endl;
  std::cout << std::endl << rob1.JacobianLeft() << std::endl;

  Eigen::Vector<double, 6> angle_rate;
  angle_rate << 0.05, 0.03, 0.02, 0.01, 0.06, 0.03;
  Eigen::Vector<double, 3> edf_p(0, 0, 1);

  Eigen::Vector<double, 3> end_effctor_vel_right = rob1.End_effctor_vel<LegsRobot<double>::LEG::RIGHT>(edf_p, angle_rate);
  Eigen::Vector<double, 3> end_effctor_vel_left = rob1.End_effctor_vel<LegsRobot<double>::LEG::LEFT>(edf_p, angle_rate);
  Eigen::Vector<double, 3> end_effctor_vel_right_direct = rob1.End_effctor_vel_direct<LegsRobot<double>::LEG::RIGHT>(edf_p, angle_rate);
  Eigen::Vector<double, 3> end_effctor_vel_left_direct = rob1.End_effctor_vel_direct<LegsRobot<double>::LEG::LEFT>(edf_p, angle_rate);

  std::cout << std::endl
    << end_effctor_vel_right
    << std::endl;
  std::cout << std::endl <<
    end_effctor_vel_left
    << std::endl;
  std::cout << std::endl
    << end_effctor_vel_right_direct
    << std::endl;
  std::cout << std::endl
    << end_effctor_vel_left_direct
    << std::endl;

  EXPECT_TRUE(end_effctor_vel_right.isApprox(end_effctor_vel_right_direct, 1e-6));
  EXPECT_TRUE(end_effctor_vel_left.isApprox(end_effctor_vel_left_direct, 1e-6));

  Eigen::Vector<double, 6> angle_rate_cal;
  angle_rate_cal = rob1.Angle_vel_edf<LegsRobot<double>::LEG::RIGHT>(edf_p, end_effctor_vel_right_direct);
  std::cout << std::endl << angle_rate_cal << std::endl;

  Eigen::Vector<double, 3> edf_vel_cal = rob1.End_effctor_vel<LegsRobot<double>::LEG::RIGHT>(edf_p, angle_rate_cal);
  std::cout << std::endl
    << edf_vel_cal
    << std::endl;
  
  EXPECT_TRUE(edf_vel_cal.isApprox(end_effctor_vel_right, 1e-6));
}