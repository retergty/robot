#include "LegsRobot.hpp"
#include "WalkPatternGen.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

template <typename scalar>
constexpr scalar ANGLE_OFFSET = 100;

template<typename scalar>
constexpr scalar LEFT_LEG_ANGLE_ZERO_POINT[5] = { 100,60,200,55,100 };
template<typename scalar>
constexpr scalar RIGHT_LEG_ANGLE_ZERO_POINT[5] = { 100,60,200,55,100 };
template<typename scalar>
constexpr scalar LEFT_LEG_ANGLE_COF[5] = { 1,-1,-1,-1,1 };
template<typename scalar>
constexpr scalar RIGHT_LEG_ANGLE_COF[5] = { -1,-1,-1,-1,-1 };

template<typename scalar>
constexpr scalar LEJU_COM_Z_FALL_DOWN = 0.018;

constexpr size_t SPEED_DEFAULT = 20;
constexpr unsigned short STIFFNESS_DEFAULT = 50;

template<typename scalar>
inline scalar LejuMotorAngle(scalar angle, scalar angle_cof, scalar angle_zero_point) { return angle_cof * angle + angle_zero_point; };

// generate leju robot IDE happy angle string
template <typename scalar>
std::string GenerateFormatAngle(const Eigen::Vector<scalar, 5>& right_leg, const Eigen::Vector<scalar, 5>& left_leg,
  const Eigen::Vector<scalar, 3>& right_hand, const Eigen::Vector<scalar, 3>& left_hand,
  const Eigen::Vector<scalar, 3>& remote_motor, const size_t speed = SPEED_DEFAULT)
{
  std::ostringstream oss;
  scalar angle_offset = ANGLE_OFFSET<scalar>;
  oss << "SPEED " << speed << std::endl;
  oss << "MOTORA,"
    << round(LejuMotorAngle(left_leg(4), LEFT_LEG_ANGLE_COF<scalar>[4], LEFT_LEG_ANGLE_ZERO_POINT<scalar>[4])) << ","
    << round(LejuMotorAngle(left_leg(3), LEFT_LEG_ANGLE_COF<scalar>[3], LEFT_LEG_ANGLE_ZERO_POINT<scalar>[3])) << ","
    << round(LejuMotorAngle(left_leg(2), LEFT_LEG_ANGLE_COF<scalar>[2], LEFT_LEG_ANGLE_ZERO_POINT<scalar>[2])) << ","
    << round(LejuMotorAngle(left_leg(1), LEFT_LEG_ANGLE_COF<scalar>[1], LEFT_LEG_ANGLE_ZERO_POINT<scalar>[1])) << ","
    << round(LejuMotorAngle(left_leg(0), LEFT_LEG_ANGLE_COF<scalar>[0], LEFT_LEG_ANGLE_ZERO_POINT<scalar>[0])) << std::endl;
  oss << "MOTORB,"
    << round(LejuMotorAngle(right_leg(4), RIGHT_LEG_ANGLE_COF<scalar>[4], RIGHT_LEG_ANGLE_ZERO_POINT<scalar>[4])) << ","
    << round(LejuMotorAngle(right_leg(3), RIGHT_LEG_ANGLE_COF<scalar>[3], RIGHT_LEG_ANGLE_ZERO_POINT<scalar>[3])) << ","
    << round(LejuMotorAngle(right_leg(2), RIGHT_LEG_ANGLE_COF<scalar>[2], RIGHT_LEG_ANGLE_ZERO_POINT<scalar>[2])) << ","
    << round(LejuMotorAngle(right_leg(1), RIGHT_LEG_ANGLE_COF<scalar>[1], RIGHT_LEG_ANGLE_ZERO_POINT<scalar>[1])) << ","
    << round(LejuMotorAngle(right_leg(0), RIGHT_LEG_ANGLE_COF<scalar>[0], RIGHT_LEG_ANGLE_ZERO_POINT<scalar>[0])) << std::endl;
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
  oss << "RIGIDD," << right_hand(2) << "," << right_hand(1) << "," << right_hand(0) << std::endl;
  oss << "RIGIDE," << remote_motor(2) << "," << remote_motor(1) << "," << remote_motor(0) << std::endl;
  oss << "RIGEND" << std::endl << std::endl;
  return oss.str();
}

// only generate walk, keep other motor as before
template <typename scalar>
void GenerateFormatWalkTo(std::ostream& os,
  const std::vector<Eigen::Vector<scalar, 12>>& leg_angle, const Eigen::Vector<scalar, 3>& right_hand,
  const Eigen::Vector<scalar, 3>& left_hand, const Eigen::Vector<scalar, 3>& remote_motor, const size_t speed = SPEED_DEFAULT)
{
  const Eigen::Vector<unsigned short, 5>& stiffness_leg = Eigen::Vector<unsigned short, 5>::Constant(STIFFNESS_DEFAULT);
  const Eigen::Vector<unsigned short, 3>& stiffness_hand = Eigen::Vector<unsigned short, 3>::Constant(STIFFNESS_DEFAULT);
  Eigen::Vector<scalar, 5> right_leg_last = Eigen::Vector<scalar, 5>::Zero();
  Eigen::Vector<scalar, 5> left_leg_last = Eigen::Vector<scalar, 5>::Zero();
  Eigen::Vector<scalar, 5> right_leg;
  Eigen::Vector<scalar, 5> left_leg;
  os << GenerateFormatStiffness(stiffness_leg, stiffness_leg, stiffness_hand, stiffness_hand, stiffness_hand);
  const size_t jump_index = 60;
  for (size_t i = 0; i < leg_angle.size(); i += jump_index)
  {
    right_leg = (leg_angle[i].template segment<5>(1).array() * Rad2Deg<double>).matrix();
    left_leg = (leg_angle[i].template segment<5>(7).array() * Rad2Deg<double>).matrix();

    if (!isMatrixEqual(right_leg,right_leg_last,1) || !isMatrixEqual(left_leg,left_leg_last,1) || (i + jump_index >= leg_angle.size())) {
      os << GenerateFormatAngle(right_leg, left_leg, right_hand, left_hand, remote_motor, speed);
      right_leg_last = right_leg;
      left_leg_last = left_leg;
    }
  }
  return;
}

void WalkingOneStep()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 80,30,100 };
  Eigen::Vector<double, 3> remote_hand{ 128,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double> });

  std::vector<double> sx = { 0,0.02,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::RIGHT, 0.6);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/walking.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand);
  fos.close();
}

void WalkingOneStepLong()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 80,30,100 };
  Eigen::Vector<double, 3> remote_hand{ 128,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double> });

  std::vector<double> sx = { 0,0.065,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.6);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/walking_long.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand, 50);
  fos.close();

}

void RightOneStep()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 80,30,100 };
  Eigen::Vector<double, 3> remote_hand{ 128,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double> });
  std::vector<double> sx = { 0,0,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,1.5 * param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0 };
  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.7);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/right_move.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand, 40);
  fos.close();
}

void LeftOneStep()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 80,30,100 };
  Eigen::Vector<double, 3> remote_hand{ 128,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double> });
  std::vector<double> sx = { 0,0,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,1.5 * param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0 };
  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::RIGHT, 0.7);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/left_move.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand, 40);
  fos.close();
}

void RightWithHandRise()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 78,25,181 };
  Eigen::Vector<double, 3> remote_hand{ 127,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.015 });
  std::vector<double> sx = { 0,0,0,0,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,2 * param::STEP_WIDTH,param::STEP_WIDTH,2 * param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0,0,0 };
  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.7);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/right_move_rise_hand.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand, 40);
  fos.close();
}
void ForwardWithHandRise()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 78,25,181 };
  Eigen::Vector<double, 3> remote_hand{ 127,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.015 });
  std::vector<double> sx = { 0,0.05,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0 };
  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.7);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/forward_move_rise_hand.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand, 40);
  fos.close();
}
void RightForwardWithHandRise()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 78,25,181 };
  Eigen::Vector<double, 3> remote_hand{ 127,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.015 });
  std::vector<double> sx = { 0,0.01,0.01,0.01,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,2 * param::STEP_WIDTH,param::STEP_WIDTH,2 * param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0,0,0 };
  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.7);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/rightforward_move_rise_hand.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand, 40);
  fos.close();
}
void UpStairs()
{
  Eigen::Vector<double, 3> right_hand_rise{ 87,90,100 };
  Eigen::Vector<double, 3> left_hand_rise{ 79,94,98 };
  Eigen::Vector<double, 3> remote_hand_rise{ 128,73,100 };
  right_hand_rise.array() -= ANGLE_OFFSET<double>;
  left_hand_rise.array() -= ANGLE_OFFSET<double>;
  remote_hand_rise.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.075 });
  std::vector<double> sx_down = { 0.031,0.153,0 };
  std::vector<double> sy_down = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz_down = { 0,0.02,0 };
  walk.GenerateContinuousStep(sx_down, sy_down, sz_down, WalkPatternGen<double>::LEG::RIGHT);
  walk.GenerateStillStep(1.0);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition<StairsMethod<double, long double>, FivePolyMethod<double, 1, long double> >();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/upStairs.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_rise, left_hand_rise, remote_hand_rise);
  fos.close();
}

void DownStairs()
{
  Eigen::Vector<double, 3> right_hand_rise{ 87,90,100 };
  Eigen::Vector<double, 3> left_hand_rise{ 79,94,98 };
  Eigen::Vector<double, 3> remote_hand_rise{ 128,73,100 };
  right_hand_rise.array() -= ANGLE_OFFSET<double>;
  left_hand_rise.array() -= ANGLE_OFFSET<double>;
  remote_hand_rise.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.075 });
  std::vector<double> sx_down = { 0.035,0.153,0 };
  std::vector<double> sy_down = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz_down = { 0,-0.02,0 };
  walk.GenerateContinuousStep(sx_down, sy_down, sz_down, WalkPatternGen<double>::LEG::RIGHT);
  walk.GenerateStillStep(1.0);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition<StairsMethod<double, long double>, FivePolyMethod<double, 1, long double> >();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/downStairs.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_rise, left_hand_rise, remote_hand_rise);
  fos.close();
}

void DownStairs2()
{
  Eigen::Vector<double, 3> right_hand{ 80,30,100 };
  Eigen::Vector<double, 3> left_hand{ 80,30,100 };
  Eigen::Vector<double, 3> remote_hand{ 128,71,100 };
  right_hand.array() -= ANGLE_OFFSET<double>;
  left_hand.array() -= ANGLE_OFFSET<double>;
  remote_hand.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.075 });
  std::vector<double> sx_down = { 0.037,0.15,0 };
  std::vector<double> sy_down = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz_down = { 0,-0.02,0 };
  walk.GenerateContinuousStep(sx_down, sy_down, sz_down, WalkPatternGen<double>::LEG::RIGHT);
  walk.GenerateStillStep(1.0);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition<StairsMethod<double, long double>, FivePolyMethod<double, 1, long double> >();
  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();

  std::ofstream fos("WalkGen/downStairs2.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand, left_hand, remote_hand, 15);
  fos.close();
}

void MoveGrab()
{
  Eigen::Vector<double, 3> right_hand_grab{ 62,18,171 };
  Eigen::Vector<double, 3> left_hand_grab{ 65,10,173 };
  Eigen::Vector<double, 3> remote_hand_grab{ 122,74,100 };
  right_hand_grab.array() -= ANGLE_OFFSET<double>;
  left_hand_grab.array() -= ANGLE_OFFSET<double>;
  remote_hand_grab.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double> });
  std::vector<double> sx = { 0,0.06,0.06,0.06,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::RIGHT, 0.85);
  walk.GenerateStillStep(0.2);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/move_grab.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_grab, left_hand_grab, remote_hand_grab);
  fos.close();
}
void MoveGrabWideRightLegFirst()
{
  Eigen::Vector<double, 3> right_hand_grab{ 62,18,171 };
  Eigen::Vector<double, 3> left_hand_grab{ 65,10,173 };
  Eigen::Vector<double, 3> remote_hand_grab{ 122,74,100 };
  right_hand_grab.array() -= ANGLE_OFFSET<double>;
  left_hand_grab.array() -= ANGLE_OFFSET<double>;
  remote_hand_grab.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.025 });
  std::vector<double> sx = { 0,0.06,0.06,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH + 0.01,param::STEP_WIDTH + 0.01,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::RIGHT, 0.85);
  walk.GenerateStillStep(0.2);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/move_grab_wide_right_leg_first.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_grab, left_hand_grab, remote_hand_grab);
  fos.close();
}
void MoveGrabWidLeftLegFirst()
{
  Eigen::Vector<double, 3> right_hand_grab{ 62,18,171 };
  Eigen::Vector<double, 3> left_hand_grab{ 65,10,173 };
  Eigen::Vector<double, 3> remote_hand_grab{ 122,74,100 };
  right_hand_grab.array() -= ANGLE_OFFSET<double>;
  left_hand_grab.array() -= ANGLE_OFFSET<double>;
  remote_hand_grab.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.025 });
  std::vector<double> sx = { 0,0.06,0.06,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH + 0.01,param::STEP_WIDTH + 0.01,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.85);
  walk.GenerateStillStep(0.2);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/move_grab_wide_left_leg_first.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_grab, left_hand_grab, remote_hand_grab);
  fos.close();
}
void MoveRightGrab()
{
  Eigen::Vector<double, 3> right_hand_grab{ 47,17,179 };
  Eigen::Vector<double, 3> left_hand_grab{ 38,34,179 };
  Eigen::Vector<double, 3> remote_hand_grab{ 121,80,99 };
  right_hand_grab.array() -= ANGLE_OFFSET<double>;
  left_hand_grab.array() -= ANGLE_OFFSET<double>;
  remote_hand_grab.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.015 });
  std::vector<double> sx = { 0,0,0,0,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,1.5 * param::STEP_WIDTH,param::STEP_WIDTH,1.5 * param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.8);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/moveright_grab.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_grab, left_hand_grab, remote_hand_grab, 20);
  fos.close();
}
void MoveLeftGrab()
{
  Eigen::Vector<double, 3> right_hand_grab{ 47,17,179 };
  Eigen::Vector<double, 3> left_hand_grab{ 38,34,179 };
  Eigen::Vector<double, 3> remote_hand_grab{ 121,80,99 };
  right_hand_grab.array() -= ANGLE_OFFSET<double>;
  left_hand_grab.array() -= ANGLE_OFFSET<double>;
  remote_hand_grab.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.015 });
  std::vector<double> sx = { 0,0,0,0,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,1.5 * param::STEP_WIDTH,param::STEP_WIDTH,1.5 * param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::RIGHT, 0.8);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/moveleft_grab.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_grab, left_hand_grab, remote_hand_grab, 20);
  fos.close();
}

void MoveForwardGrab()
{
  Eigen::Vector<double, 3> right_hand_grab{ 47,17,179 };
  Eigen::Vector<double, 3> left_hand_grab{ 38,34,179 };
  Eigen::Vector<double, 3> remote_hand_grab{ 121,80,99 };
  right_hand_grab.array() -= ANGLE_OFFSET<double>;
  left_hand_grab.array() -= ANGLE_OFFSET<double>;
  remote_hand_grab.array() -= ANGLE_OFFSET<double>;

  WalkPatternGen<double> walk({ 0,0,param::COM_Z - LEJU_COM_Z_FALL_DOWN<double>-0.015 });
  std::vector<double> sx = { 0,0.045,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = { 0,0,0 };

  walk.GenerateContinuousStep(sx, sy, sz, WalkPatternGen<double>::LEG::LEFT, 0.8);
  walk.GenerateStillStep(0.1);
  walk.UpdateState();
  walk.GenerateTrajectoryPosition();

  std::vector<Eigen::Vector<double, 12>> walk_angle = walk.GetWalkAngle();
  std::ofstream fos("WalkGen/moveforward_grab.src", std::ios::out);
  GenerateFormatWalkTo(fos, walk_angle, right_hand_grab, left_hand_grab, remote_hand_grab, 20);
  fos.close();
}

int main()
{
  WalkingOneStep();
  WalkingOneStepLong();

  RightOneStep();
  LeftOneStep();

  RightWithHandRise();
  ForwardWithHandRise();
  RightForwardWithHandRise();

  UpStairs();
  DownStairs();
  DownStairs2();

  MoveGrab();
  MoveGrabWideRightLegFirst();
  MoveGrabWidLeftLegFirst();

  MoveRightGrab();
  MoveLeftGrab();
  MoveForwardGrab();
  return 0;
}