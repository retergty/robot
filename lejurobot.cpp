#include "LegsRobot.hpp"
#include "WalkPatternGen.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

template <typename scalar>
constexpr scalar ANGLE_OFFSET = 100;

constexpr size_t SPEED_DEFAULT = 40;
constexpr unsigned short STIFFNESS_DEFAULT = 40;

// generate leju robot IDE happy angle string
template <typename scalar>
std::string GenerateFormatAngle(const Eigen::Vector<scalar, 5>& right_leg, const Eigen::Vector<scalar, 5>& left_leg,
  const Eigen::Vector<scalar, 3>& right_hand, const Eigen::Vector<scalar, 3>& left_hand,
  const Eigen::Vector<scalar, 3>& remote_motor, const size_t speed = SPEED_DEFAULT)
{
  std::ostringstream oss;
  scalar angle_offset = ANGLE_OFFSET<scalar>;
  oss << "SPEED " << speed << std::endl;
  oss << "MOTORA," << round(left_leg(4) + angle_offset) << "," << round(left_leg(3) + angle_offset) << "," << round(left_leg(2) + angle_offset) << "," << round(left_leg(1) + angle_offset) << "," << round(left_leg(0) + angle_offset) << std::endl;
  oss << "MOTORB," << round(right_leg(4) + angle_offset) << "," << round(right_leg(3) + angle_offset) << "," << round(right_leg(2) + angle_offset) << "," << round(right_leg(1) + angle_offset) << "," << round(right_leg(0) + angle_offset) << std::endl;
  oss << "MOTORC," << round(left_hand(2) + angle_offset) << "," << round(left_hand(1) + angle_offset) << "," << round(left_hand(0) + angle_offset) << std::endl;
  oss << "MOTORD," << round(right_hand(2) + angle_offset) << "," << round(right_hand(1) + angle_offset) << "," << round(right_hand(0) + angle_offset) << std::endl;
  oss << "MOTORE," << round(remote_motor(2) + angle_offset) << "," << round(remote_motor(1) + angle_offset) << "," << round(remote_motor(0) + angle_offset) << std::endl;
  oss << "WAIT" << std::endl;
  return oss.str();
}

// generate leju robot IDE happy stiffness string
std::string GenerateFormatStiffness(const Eigen::Vector<unsigned short, 5>& right_leg, const Eigen::Vector<unsigned short, 5>& left_leg,
  const Eigen::Vector<unsigned short, 3>& right_hand, const Eigen::Vector<unsigned short, 3>& left_hand,
  const Eigen::Vector<unsigned short, 3>& remote_motor)
{
  std::ostringstream oss;
  oss << "RIGIDA," << left_leg(4) << "," << left_leg(3) << "," << left_leg(2) << "," << left_leg(1) << "," << left_leg(0) << std::endl;
  oss << "RIGIDB," << right_leg(4) << "," << right_leg(3) << "," << right_leg(2) << "," << right_leg(1) << "," << right_leg(0) << std::endl;
  oss << "RIGIDC," << left_hand(2) << "," << left_hand(1) << "," << left_hand(0) << std::endl;
  oss << "RIGIDD," << right_hand(2) << "," << right_hand << "," << right_hand(0) << std::endl;
  oss << "RIGIDE," << remote_motor(2) << "," << remote_motor(1) << "," << remote_motor(0) << std::endl;
  oss << "RIGEND" << std::endl;
  return oss.str();
}

template <typename scalar>
void GenerateFormatBodyTo(std::ostream& os,
  const std::vector<Eigen::Vector<scalar, 12>>& leg_angle, const std::vector<Eigen::Vector<scalar, 3>>& right_hand,
  const std::vector<Eigen::Vector<scalar, 3>>& left_hand, const std::vector<Eigen::Vector<scalar, 3>>& remote_motor)
{
  assert(leg_angle.size() == left_hand.size());
  assert(leg_angle.size() == right_hand.size());
  assert(leg_angle.size() == remote_motor.size());
  const Eigen::Vector<unsigned short, 5>& stiffness_leg = Eigen::Vector<unsigned short, 5>::Constant(STIFFNESS_DEFAULT);
  const Eigen::Vector<unsigned short, 3>& stiffness_hand = Eigen::Vector<unsigned short, 3>::Constant(STIFFNESS_DEFAULT);
  os << GenerateFormatStiffness(stiffness_leg, stiffness_leg, stiffness_hand, stiffness_hand, stiffness_hand);
  for (size_t i = 0; i < leg_angle.size(); ++i)
  {
    os << GenerateFormatAngle(leg_angle[i].template segment<5>(1), leg_angle[i].template segment<5>(7), right_hand[i], left_hand[i], remote_motor[i]);
  }
  return;
}
// only generate walk, keep other motor as before
template <typename scalar>
void GenerateFormatWalkTo(std::ostream& os,
  const std::vector<Eigen::Vector<scalar, 12>>& leg_angle, const Eigen::Vector<scalar, 3>& right_hand,
  const Eigen::Vector<scalar, 3>& left_hand, const Eigen::Vector<scalar, 3>& remote_motor)
{
  const Eigen::Vector<unsigned short, 5>& stiffness_leg = Eigen::Vector<unsigned short, 5>::Constant(STIFFNESS_DEFAULT);
  const Eigen::Vector<unsigned short, 3>& stiffness_hand = Eigen::Vector<unsigned short, 3>::Constant(STIFFNESS_DEFAULT);
  os << GenerateFormatStiffness(stiffness_leg, stiffness_leg, stiffness_hand, stiffness_hand, stiffness_hand);
  for (size_t i = 0; i < leg_angle.size(); ++i)
  {
    os << GenerateFormatAngle(leg_angle[i].template segment<5>(1).eval(), leg_angle[i].template segment<5>(7).eval(), right_hand, left_hand, remote_motor);
  }
  return;
}
int main()
{
  WalkPatternGen<double> walk1;
  walk1.GenerateAStep();
  walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);
  walk1.UpdateState();
  walk1.GenerateTrajectoryPosition();

  Eigen::Vector<double, 3> right_hand{ -20,-70,0 };
  Eigen::Vector<double, 3> left_hand{ -20,-70,0 };
  Eigen::Vector<double, 3> remote_hand{ 28,-29,0 };
  std::ofstream fos("WalkGen/walking.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk1.GetWalkAngle(), right_hand, left_hand, remote_hand);
  fos.close();
  return 0;
}