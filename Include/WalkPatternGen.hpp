#pragma once
#include "Dense"
#include "Geometry"
#include "LegsRobot.hpp"
#include <iostream>
#include <fstream>
#include "prvctl_parm.hpp"
#include <utility>
#include <type_traits>
#include <iterator>
#include "RobotParamDef.hpp"
#include <cmath>
#include "HelperFunction.hpp"
#include "TrajectoryGenMethod.hpp"

// generate smoothstep behind trajectory, use 4-order polynomial,Tsmooth is total smoothstep time,new_val is new step val.
// y = a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4
template<typename scalar>
void SmoothStep(std::vector<scalar>& trajectory, const scalar Tsmooth, const scalar new_val, const scalar sample_time)
{
  const scalar old_val = trajectory.back();
  const scalar T = Tsmooth;
  Eigen::Matrix<scalar, 5, 5> A{
    {1,0,0,0,0},
    {1,T / 2,T * T / 4,T * T * T / 8,T * T * T * T / 16},
    {1,T,T * T,T * T * T,T * T * T * T},
    {0,1,0,0,0},
    {0,1,2 * T,3 * T * T,4 * T * T * T}
  };
  Eigen::Vector<scalar, 5> b{ old_val,(new_val + old_val) / 2,new_val,0,0 };
  Eigen::Vector<scalar, 5> a = A.colPivHouseholderQr().solve(b);

  scalar t = sample_time;

  while (t < Tsmooth || isEqual(t, Tsmooth))
  {
    scalar y = a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t;
    trajectory.push_back(y);
    t += sample_time;
  }
}
template<typename scalar>
class WalkPatternGen
{
  // walk pattern
public:
  constexpr static scalar Tstep = param::STEP_TIME; //total step time
  constexpr static scalar Tstart = param::START_SCALAR * Tstep;// a step start time
  constexpr static scalar Tdbl = param::DBL_SCALAR * Tstep; // double support time
  constexpr static scalar Trest = param::REST_SCALAR * Tstep; //step rest time
  constexpr static scalar Sx = param::STEP_LENGTH;  //step forward 
  constexpr static scalar Sy = param::STEP_WIDTH; //step width
  constexpr static scalar Sz = param::STEP_HEIGHT; // step height
  constexpr static long SwingLegPreserveIndex = std::floor(Tstart / PREVIEW_CONTROL_SAMPLE_TIME * 0.1);//use to generate swing leg trajectory, leave some times between single leg period and double leg period
  constexpr static long ComZMovingWaitIndex = std::floor(Tstep / PREVIEW_CONTROL_SAMPLE_TIME * 0.3); // leave some times, before com z start moving
  constexpr static long ComZMovingPreserveIndex = std::floor(Tstep / PREVIEW_CONTROL_SAMPLE_TIME * 0.3); // leave some times, after com z finish moving

  using Vector3 = Eigen::Vector<scalar, 3>;
  using Matrix33 = Eigen::Matrix<scalar, 3, 3>;
  enum LEG {
    RIGHT,
    LEFT,
    TWO
  };

  WalkPatternGen(const Vector3& com_position = { 0,0,param::COM_Z }, const Matrix33& com_rotation = Matrix33::Identity()) : _legrobot(com_position, com_rotation) {
    _Zc = com_position(2);
    _C_(0) = 1;
    _C_(1) = 0;
    _C_(2) = -com_position(2) / PREVIEW_CONTROL_ACC_G;

    //give target z a init val equal to support leg 
    const Vector3& last_joint = _legrobot.rightLeg().back().position();
    _target_z.push_back(last_joint(2));

    //give reference zmp a init val
    _ref_zmp.x.push_back(com_position(0));
    _ref_zmp.y.push_back(com_position(1));
  }

  //generate a robot step reference zmp,first move zmp to support leg,second move zmp to forward, finally move zmp to center
  // leg is support leg
  void GenerateAStep(const scalar sx = Sx, const scalar sy = Sy, const scalar sz = Sz, const LEG sup_leg = LEG::RIGHT, const scalar time_step = Tstep)
  {
    std::vector<scalar> target_sx{ 0,sx,0 };
    std::vector<scalar> target_sy{ sy / 2,sy,sy / 2 };
    std::vector<scalar> target_sz{ 0,sz,0 };
    GenerateContinuousStep(target_sx, target_sy, target_sz, sup_leg, time_step);
  }

  //generate a robot step reference zmp,use foot support plant, first move zmp to support leg,second move zmp to forward, finally move zmp to center
  // leg is support leg
  void GenerateAStepAggressive(const scalar sx = Sx, const scalar sy = Sy, const scalar sz = Sz, const LEG sup_leg = LEG::RIGHT, const scalar time_step = Tstep)
  {
    std::vector<scalar> target_sx{ param::FOOT_SUPPORT_PLANT,sx,0 };
    std::vector<scalar> target_sy{ sy / 2,sy,sy / 2 };
    std::vector<scalar> target_sz{ 0,sz,0 };
    GenerateContinuousStep(target_sx, target_sy, target_sz, sup_leg, time_step);
  }

  //generate continuous step
  // leg is first step support leg
  void GenerateContinuousStep(const std::vector<scalar>& sx, const std::vector<scalar>& sy, const std::vector<scalar>& sz, const LEG first_sup_leg, const scalar time_step = Tstep) {
    assert(sx.size() == sy.size());
    assert(sx.size() == sz.size());
    
    const scalar time_start = time_step * param::START_SCALAR;
    const scalar time_dbl = time_step * param::DBL_SCALAR;
    const scalar time_rest = time_step * param::REST_SCALAR;

    xy_dim<scalar> target_zmp{ _ref_zmp.x.back(), _ref_zmp.y.back() };
    scalar target_z = _target_z.back();

    if (_step_total == 0) {
      _first_support = first_sup_leg;
    }

    size_t factor = 0;
    if (first_sup_leg == LEG::LEFT) {
      ++factor;
    }

    for (size_t i = 0;i < sx.size();++i) {
      target_zmp.x += sx[i];
      if (factor % 2)
        target_zmp.y += sy[i];
      else
        target_zmp.y -= sy[i];
      ++factor;
      GenerateAStepZMP(target_zmp, time_start, time_dbl, time_rest);

      target_z += sz[i];
      _target_z.push_back(target_z);
    }
  }

  //generate still step
  void GenerateStillStep(const scalar time_still) {
    xy_dim<scalar> old_zmp{ _ref_zmp.x.back(), _ref_zmp.y.back() };
    GenerateAStepZMP(old_zmp, time_still);
    return;
  }

  //update state, use reference zmp to calculate state vector
  void UpdateState() {
    Vector3 com_init_position = _legrobot.massCenter().position();
    Vector3 x_init = { com_init_position(0),0,0 };
    Vector3 y_init = { com_init_position(1),0,0 };

    InitState(x_init, y_init);

    const size_t ref_zmp_size = _ref_zmp.x.size();
    for (size_t i = 0;i < ref_zmp_size;++i) {
      scalar zmp_x = (_C_ * _state.x[i]).value();
      scalar zmp_y = (_C_ * _state.y[i]).value();
      _zmp.x.push_back(zmp_x);
      _zmp.y.push_back(zmp_y);

      GenerateInput(i);

      Vector3 x_next = _A_ * _state.x[i] + _B_ * _input.x[i];
      Vector3 y_next = _A_ * _state.y[i] + _B_ * _input.y[i];

      _state.x.push_back(std::move(x_next));
      _state.y.push_back(std::move(y_next));
    }
  }

  //generate trajectory potition,using ref_zmp and state
  template<typename SwingLegMethod = FourPolyMethod<scalar, 3, scalar>, typename ComZMethod = KeepMethod<scalar, 1, scalar>>
  void GenerateTrajectoryPosition() {
    static_assert(std::is_base_of_v<BaseMethod<scalar, 3>, SwingLegMethod>);
    static_assert(std::is_base_of_v<BaseMethod<scalar, 1>, ComZMethod>);
    static_assert(std::is_same_v<typename SwingLegMethod::value_type, scalar>);
    static_assert(std::is_same_v<typename ComZMethod::value_type, scalar>);

    //get robot initial com,right leg,left leg
    Vector3 robot_com = _legrobot.massCenter().position();

    //get initial state and ref_zmp zmp
    xy_dim<Vector3> init_state = _state[0];

    //initial center of mass must fit robot center of mass
    assert(isEqual(init_state.x(0), robot_com(0)));
    assert(isEqual(init_state.y(0), robot_com(1)));
    assert(isEqual(_Zc, robot_com(2)));

    typename std::vector<scalar>::difference_type index = GenerateStartTrajectoryPosition();
    while ((index = GenerateAStepTrajectoryPosition<SwingLegMethod, ComZMethod>(index)) < static_cast<decltype(index)>(_ref_zmp.x.size()));
  }
  //generate walk angle
  std::vector<Eigen::Vector<scalar, 12>> GetWalkAngle() {
    std::vector<Eigen::Vector<scalar, 12>> angle;
    Eigen::Vector<scalar, 12> now_angle;
    for (size_t i = 0;i < _trajectory.com.size();++i) {
      _legrobot.Inverse_kinematics(_trajectory.com[i], _trajectory.right[i], _trajectory.left[i]);
      now_angle.template head<6>() = _legrobot.rightLegAngle();
      now_angle.template tail<6>() = _legrobot.leftLegAngle();
      angle.push_back(now_angle);
    }
    return angle;
  }

  template<bool is_x>
  inline const std::vector<scalar>& GetRefZmp() const {
    if constexpr (is_x) {
      return _ref_zmp.x;
    }
    else {
      return _ref_zmp.y;
    }
  }
  template<bool is_x>
  inline const std::vector<scalar>& GetZmp() const {
    if constexpr (is_x) {
      return _zmp.x;
    }
    else {
      return _zmp.y;
    }
  }
  template<bool is_x>
  inline const std::vector<Vector3>& GetState() const {
    if constexpr (is_x) {
      return _state.x;
    }
    else {
      return _state.y;
    }
  }
  inline const std::vector<Pose<scalar>>& GetTrajectoryCom()const {
    return _trajectory.com;
  }
  inline const std::vector<Pose<scalar>>& GetTrajectoryLeftLeg()const {
    return _trajectory.left;
  }
  inline const std::vector<Pose<scalar>>& GetTrajectoryRightLeg()const {
    return _trajectory.right;
  }
  inline const std::vector<LEG>& GetTrajectorySupportLeg()const {
    return _trajectory.support;
  }

  inline const std::vector<typename std::vector<scalar>::size_type>& GetLastSingleSupportIndex()const {
    return _last_single_support_index;
  }

  inline const std::vector<typename std::vector<scalar>::size_type>& GetLastDoubleSupportIndex()const {
    return _last_double_support_index;
  }
private:
  //clear state,but keep ref_zmp and relative var
  void ClearState() {
    _state.clear();

    _trajectory.com.clear();
    _trajectory.left.clear();
    _trajectory.right.clear();
    _trajectory.support.clear();

    _zmp.clear();

    _input.clear();
    _input_tmp.clear();

    _step_count = 0;
  }
  // init state
  // init zmp
  // init input
  void InitState(const Vector3& x, const Vector3& y) {
    // clear all previous datas
    ClearState();

    //reserve space
    size_t cap = _ref_zmp.x.size();
    _state.x.reserve(cap);
    _state.y.reserve(cap);
    _trajectory.com.reserve(cap);
    _trajectory.left.reserve(cap);
    _trajectory.right.reserve(cap);
    _trajectory.support.reserve(cap);
    _zmp.x.reserve(cap);
    _zmp.y.reserve(cap);
    _input.x.reserve(cap);
    _input.y.reserve(cap);

    //set init state
    _state.x.push_back(x);
    _state.y.push_back(y);

    //set last support index to the end
    _last_double_support_index.push_back(_ref_zmp.x.size() - 1);
    _last_single_support_index.push_back(_ref_zmp.x.size() - 1);
  }

  //generate reference zmp of single step zmp
  void GenerateAStepZMP(const xy_dim<scalar>& target_zmp, const scalar time_start = Tstart, const scalar time_dbl = Tdbl, const scalar time_rest = Trest) {
    xy_dim<scalar> old_zmp{ _ref_zmp.x.back(), _ref_zmp.y.back() };
    scalar t = _sample_time;

    if (old_zmp.x != target_zmp.x || old_zmp.y != target_zmp.y) {
      //keep old value
      while (t < time_start || isEqual(t, time_start)) {
        _ref_zmp.x.push_back(old_zmp.x);
        _ref_zmp.y.push_back(old_zmp.y);
        t += _sample_time;
      }

      _last_single_support_index.push_back(_ref_zmp.x.size() - 1);

      //smooth step
      SmoothStep(_ref_zmp.x, time_dbl, target_zmp.x, _sample_time);
      SmoothStep(_ref_zmp.y, time_dbl, target_zmp.y, _sample_time);

      _last_double_support_index.push_back(_ref_zmp.x.size() - 1);

      //keep new value
      t = _sample_time;
      while (t < time_rest || isEqual(t, time_rest)) {
        _ref_zmp.x.push_back(target_zmp.x);
        _ref_zmp.y.push_back(target_zmp.y);
        t += _sample_time;
      }

      //increase step count
      ++_step_total;
    }
    else {
      //keep old value
      while (t < time_start + time_dbl + time_rest || isEqual(t, time_start + time_dbl + time_rest)) {
        _ref_zmp.x.push_back(old_zmp.x);
        _ref_zmp.y.push_back(old_zmp.y);
        t += _sample_time;
      }
    }
  }

  // Generate control input, using x y state, zmp in this time, reference zmp in this and future time
  // INPOTANT ASSUMPTION! Inputs are generated in sequence.
  void GenerateInput(const size_t index) {
    _input_tmp.x += -_K_(0) * (_zmp.x[index] - _ref_zmp.x[index]);
    _input_tmp.y += -_K_(0) * (_zmp.y[index] - _ref_zmp.y[index]);
    _input.x[index] = _input_tmp.x - _K_.template tail<3>().dot(_state.x[index]);
    _input.y[index] = _input_tmp.y - _K_.template tail<3>().dot(_state.y[index]);
    for (size_t i = 0;i < _prv_num;++i) {
      _input.x[index] -= _G_[i] * _ref_zmp.x[std::min(i + index + 1, _ref_zmp.x.size() - 1)];
      _input.y[index] -= _G_[i] * _ref_zmp.y[std::min(i + index + 1, _ref_zmp.y.size() - 1)];
    }
  }


  //generate state trajetory
  //the difference with normal walking is that the initial zmp always is not in right(or left) leg support.
  //the moving starts in double leg support
  //return next moving start index
  typename std::vector<scalar>::difference_type GenerateStartTrajectoryPosition() {

    size_t support_index = _last_double_support_index[0];

    Pose<scalar> robot_right_last = _legrobot.rightLeg().back();
    Pose<scalar> robot_left_last = _legrobot.leftLeg().back();

    _now_support = _first_support;;

    const Matrix33 robot_rotation = _legrobot.massCenter().rotation();
    //generate trajectory
    for (size_t i = 0;i <= support_index;i++) {
      xy_dim<Vector3> now_state = _state[i];
      Vector3 now_com_position = { now_state.x(0),now_state.y(0),_Zc };
      _trajectory.com.emplace_back(now_com_position, robot_rotation);
      _trajectory.right.push_back(robot_right_last);
      _trajectory.left.push_back(robot_left_last);
      _trajectory.support.push_back(LEG::TWO);
    }

    //start step is a step
    ++_step_count;

    return support_index + 1;
  }

  // generate a step trajectory position,keep rotation
  // zmp_begin_index is step start index
  // return next step start index
  template<typename SwingLegMethod, typename ComZMethod>
  typename std::vector<scalar>::difference_type
    GenerateAStepTrajectoryPosition(typename std::vector<scalar>::difference_type zmp_begin_index) {

    auto last_old_index = _last_single_support_index[_step_count];
    auto next_steady_index = _last_double_support_index[_step_count];

    if (last_old_index < (_ref_zmp.x.size() - 1)) {

      Eigen::Vector<scalar, 3> target_foot_place;
      xy_dim<scalar> zmp = _ref_zmp[next_steady_index];
      if (_step_count == _step_total - 1) {
        //last step
        Vector3 support_foot_position;
        if (_now_support == LEG::LEFT) {
          support_foot_position = _trajectory.left.back().position();
        }
        else {
          support_foot_position = _trajectory.right.back().position();
        }
        // last step always in robot center
        target_foot_place(0) = 2 * zmp.x - support_foot_position(0);
        target_foot_place(1) = 2 * zmp.y - support_foot_position(1);
      }
      else {
        target_foot_place(0) = zmp.x;
        target_foot_place(1) = zmp.y;
      }
      target_foot_place(2) = _target_z[_step_count + 1];

      Eigen::Vector<scalar, 3> swing_leg_init;

      if (_now_support == LEG::LEFT) {
        swing_leg_init = _trajectory.right.back().position();
      }
      else if (_now_support == LEG::RIGHT) {
        swing_leg_init = _trajectory.left.back().position();
      }

      SwingLegMethod swing_method(swing_leg_init, target_foot_place, zmp_begin_index, last_old_index - SwingLegPreserveIndex);

      const scalar com_z_start_val = _legrobot.massCenter().position()(2);
      Eigen::Vector<scalar, 1> com_z_init{ com_z_start_val + _target_z[_step_count] };
      Eigen::Vector<scalar, 1> com_z_target{ com_z_start_val + _target_z[_step_count + 1] };
      ComZMethod comz_method(com_z_init, com_z_target, zmp_begin_index + ComZMovingWaitIndex, next_steady_index - ComZMovingPreserveIndex);

      GenerateSingleFootSupportTrajectoryPosition(zmp_begin_index, last_old_index, swing_method, comz_method);
      GenerateDoubleFootSupportTrajectoryPosition(last_old_index + 1, next_steady_index, comz_method);
      ++_step_count;
    }
    else {
      //no moving
      Eigen::Vector<scalar, 1> comz_now{ _trajectory.com.back().position()(2) };

      KeepMethod<scalar, 1> comz(comz_now, comz_now, zmp_begin_index, last_old_index);
      GenerateStillTrajectoryPosition(zmp_begin_index, last_old_index, comz);
    }

    return next_steady_index + 1;
  }

  void GenerateSingleFootSupportTrajectoryPosition(
    typename std::vector<scalar>::difference_type zmp_start_index,
    typename std::vector<scalar>::difference_type zmp_end_index,
    const BaseMethod<scalar, 3>& swing_leg_method,
    const BaseMethod<scalar, 1>& comz_method
  ) {
    //generate single foot support period
    Vector3 swing_leg_position;
    Pose<scalar> support_leg_pose;

    if (_now_support == LEG::LEFT) {
      support_leg_pose = _trajectory.left.back();
    }
    else if (_now_support == LEG::RIGHT) {
      support_leg_pose = _trajectory.right.back();
    }

    const Matrix33 robot_rotation = _legrobot.massCenter().rotation();
    for (auto i = zmp_start_index;i <= zmp_end_index;++i) {
      if (_now_support == LEG::LEFT) {
        _trajectory.right.emplace_back(swing_leg_method.NowVal(i), robot_rotation);
        _trajectory.left.push_back(support_leg_pose);
      }
      else if (_now_support == LEG::RIGHT) {
        _trajectory.left.emplace_back(swing_leg_method.NowVal(i), robot_rotation);
        _trajectory.right.push_back(support_leg_pose);
      }
      xy_dim<Vector3> now_state = _state[i];
      Vector3 now_com_position{ now_state.x(0),now_state.y(0),comz_method.NowVal(i).value() };
      _trajectory.com.emplace_back(now_com_position, robot_rotation);
      _trajectory.support.push_back(_now_support);
    }
  }

  //generate double support period
  void GenerateDoubleFootSupportTrajectoryPosition(
    typename std::vector<scalar>::difference_type zmp_start_index,
    typename std::vector<scalar>::difference_type zmp_end_index,
    const BaseMethod<scalar, 1>& comz_method
  ) {
    Pose<scalar> right_last_pose = _trajectory.right.back();
    Pose<scalar> left_last_pose = _trajectory.left.back();

    Matrix33 robot_rotation = _trajectory.com.back().rotation();
    for (auto i = zmp_start_index;i <= zmp_end_index;++i) {
      xy_dim<Vector3> now_state = _state[i];
      Vector3 now_com_position = { now_state.x(0),now_state.y(0),comz_method.NowVal(i).value() };
      _trajectory.com.emplace_back(now_com_position, robot_rotation);
      _trajectory.right.push_back(right_last_pose);
      _trajectory.left.push_back(left_last_pose);
      _trajectory.support.push_back(LEG::TWO);
    }
    //support leg exchange
    _now_support = (_now_support == LEG::LEFT) ? LEG::RIGHT : LEG::LEFT;
  }

  //Generate Still trajectory
  //do not need to calculate foot placement, but center of mass may be move because last step moving may not be finish
  void GenerateStillTrajectoryPosition(
    typename std::vector<scalar>::difference_type zmp_start_index,
    typename std::vector<scalar>::difference_type zmp_end_index,
    const BaseMethod<scalar, 1>& comz_method) {
    Pose<scalar> right_last_pose = _trajectory.right.back();
    Pose<scalar> left_last_pose = _trajectory.left.back();

    Matrix33 robot_rotation = _trajectory.com.back().rotation();
    for (auto i = zmp_start_index;i <= zmp_end_index;++i) {
      xy_dim<Vector3> now_state = _state[i];
      Vector3 now_com_position = { now_state.x(0),now_state.y(0),comz_method.NowVal(i).value() };

      _trajectory.com.emplace_back(now_com_position, robot_rotation);
      _trajectory.right.push_back(right_last_pose);
      _trajectory.left.push_back(left_last_pose);
      _trajectory.support.push_back(LEG::TWO);
    }
  }

  // simulation parmeter
  constexpr static scalar _sample_time{ PREVIEW_CONTROL_SAMPLE_TIME };
  scalar _Zc;
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_A_ROW, PREVIEW_CONTROL_A_COL> _A_{ PREVIEW_CONTROL_A_DATA };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_B_ROW, PREVIEW_CONTROL_B_COL> _B_{ PREVIEW_CONTROL_B_DATA };
  Eigen::Matrix<scalar, PREVIEW_CONTROL_C_ROW, PREVIEW_CONTROL_C_COL> _C_{ PREVIEW_CONTROL_C_DATA };
  const size_t _prv_num{ PREVIEW_CONTROL_PREVIEW_NUMBER };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_G_ROW, PREVIEW_CONTROL_G_COL>  _G_{ PREVIEW_CONTROL_G_DATA };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_K_ROW, PREVIEW_CONTROL_K_COL> _K_{ PREVIEW_CONTROL_K_DATA };

  LegsRobot<scalar> _legrobot;

  xy_dim<std::vector<scalar>> _ref_zmp;   // reference zmp
  xy_dim<std::vector<scalar>> _zmp;   // zmp
  xy_dim<std::vector<scalar>> _input; // input
  xy_dim<scalar> _input_tmp{ 0,0 }; //use to optimatize the calculation of inputs

  std::vector<typename std::vector<scalar>::size_type> _last_single_support_index; // last single foot support index in ist step, used in walk generation
  std::vector<typename std::vector<scalar>::size_type> _last_double_support_index; // last double foot support index in ist step,used in walk generation
  std::vector<scalar> _target_z; // reference z in this step

  size_t _step_total = 0; //count for how many times _ref_zmp changed
  size_t _step_count = 0;//count for how many times walk patterns have changed
  // com and last left right joint trajectory
  struct
  {
    std::vector<Pose<scalar>> com;
    std::vector<Pose<scalar>> right;
    std::vector<Pose<scalar>> left;
    std::vector<LEG> support; //support leg
  }_trajectory;
  LEG _now_support; //last support leg
  LEG _first_support; //first support leg

  //space state com state
  xy_dim<std::vector<Vector3>> _state; //x,vx,ax y,vy,ay

};
