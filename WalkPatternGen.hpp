#pragma once
#include "Dense"
#include "Geometry"
#include "LegsRobot.hpp"
#include <iostream>
#include <fstream>
#include "prvctl_parm.hpp"
#include <utility>
#include <type_traits>
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

  while (t <= Tsmooth)
  {
    scalar y = a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t;
    trajectory.push_back(y);
    t += sample_time;
  }
}

template<typename T, typename D = int>
constexpr bool has_clear = false;

template<typename T>
constexpr bool has_clear<T, decltype(std::declval<T>().clear(), int())> = true;

template<typename scalar>
class WalkPatternGen
{
private:
  // walk pattern
  template<typename Type>
  struct xy_dim
  {
    void clear() {
      if constexpr (has_clear<Type>) {
        x.clear();
        y.clear();
      }
      else {
        x = 0;
        y = 0;
      }
    }
    Type x;
    Type y;
  };
public:
  constexpr static scalar Tstep = 2; //total step time
  constexpr static scalar Tstart = 0.3 * Tstep;// a step start time
  constexpr static scalar Tdbl = 0.3 * Tstep; // double support time
  constexpr static scalar Trest = 0.4 * Tstep; //step rest time

  //generate reference zmp of single step 
  void GenerateAStepZMP(const xy_dim<scalar>& target_zmp, const scalar time_start = Tstart, const scalar time_dbl = Tdbl, const scalar time_rest = Trest) {
    xy_dim<scalar> old_zmp{ .x = _ref_zmp.x.back(),.y = _ref_zmp.y.back() };
    scalar t = _sample_time;

    //keep old value
    while (t <= Tstart) {
      _ref_zmp.x.push_back(old_zmp.x);
      _ref_zmp.y.push_back(old_zmp.y);
      t += _sample_time;
    }

    //smooth step
    SmoothStep(_ref_zmp.x, time_dbl, target_zmp.x, _sample_time);
    SmoothStep(_ref_zmp.y, time_dbl, target_zmp.y, _sample_time);

    //keep new value
    t = _sample_time;
    while (t <= Trest) {
      _ref_zmp.x.push_back(target_zmp.x);
      _ref_zmp.y.push_bask(target_zmp.y);
      t += _sample_time;
    }
  }

  // Generate control input, using x y state, zmp in this time, reference zmp in this and future time
  // INPOTANT ASSUMPTION! Inputs are generated in sequence.
  void GenerateInput(const size_t index) {
    _input_tmp.x += -_K(0) * (_zmp.x[index] - _ref_zmp.x[index]);
    _input_tmp.y += -_K(0) * (_zmp.y[index] - _ref_zmp.y[index]);
    _input.x[index] = _input_tmp.x - _K.template tail<3>().dot(_state.x[index]).eval();
    _input.y[index] = _input_tmp.y - _K.template tail<3>().dot(_state.y[index]).eval();
    for (size_t i = 0;i < _prv_num;++i) {
      _input.x[index] -= _G[i] * _ref_zmp.x[std::min(i + index + 1, _ref_zmp.x.size() - 1)];
      _input.y[index] -= _G[i] * _ref_zmp.y[std::min(i + index + 1, _ref_zmp.y.size() - 1)];
    }
  }

  //update state, use reference zmp to calculate state vector
  void UpdateState() {
    InitState(Eigen::Vector3<scalar>::Zero(), Eigen::Vector3<scalar>::Zero());

    const size_t ref_zmp_size = _ref_zmp.x.size();
    for (size_t i = 0;i < ref_zmp_size;++i) {
      scalar zmp_x = (_C * _state.x[i]).eval();
      scalar zmp_y = (_C * _state.y[i]).eval();
      _zmp.x.pushback(zmp_x);
      _zmp.y.pushback(zmp_y);

      GenerateInput(i);

      Eigen::Vector3<scalar> x_next = _A * _state.x[i] + _B * _input.x[i];
      Eigen::Vector3<scalar> y_next = _A * _state.y[i] + _B * _input.y[i];

      _state.x.pushback(std::move(x_next));
      _state.y.pushback(std::move(y_next));
    }
  }
private:
  // init state
  // init zmp
  // init input
  void InitState(const Eigen::Vector3<scalar>& x, const Eigen::Vector3<scalar>& y) {
    // clear all previous datas
    _state.clear();

    _trajectory.com.clear();
    _trajectory.left.clear();
    _trajectory.right.clear();

    _zmp.clear();

    _input.clear();
    _input_tmp.clear();

    //reserve space
    size_t cap = _ref_zmp.x.size();
    _state.x.reserve(cap);
    _state.y.reserve(cap);
    _trajectory.com.reserve(cap);
    _trajectory.left.reserve(cap);
    _trajectory.right.reserve(cap);
    _zmp.x.reserve(cap);
    _zmp.y.reserve(cap);
    _input.x.reserve(cap);
    _input.y.reserve(cap);

    //set init state
    _state.x.pushback(x);
    _state.y.pushback(y);
  }

  // simulation parmeter
  const scalar _sample_time{ PREVIEW_CONTROL_SAMPLE_TIME };
  const scalar _Zc{ PREVIEW_CONTROL_COM_Z };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_A_ROW, PREVIEW_CONTROL_A_COL> _A{ PREVIEW_CONTROL_A_DATA };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_B_ROW, PREVIEW_CONTROL_B_COL> _B{ PREVIEW_CONTROL_B_DATA };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_C_ROW, PREVIEW_CONTROL_C_COL> _C{ PREVIEW_CONTROL_C_DATA };
  const size_t _prv_num{ PREVIEW_CONTROL_PREVIEW_NUMBER };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_G_ROW, PREVIEW_CONTROL_G_COL>  _G{ PREVIEW_CONTROL_G_DATA };
  const Eigen::Matrix<scalar, PREVIEW_CONTROL_K_ROW, PREVIEW_CONTROL_K_COL> _K{ PREVIEW_CONTROL_K_DATA };
  LegsRobot<scalar> _legrobot;

  xy_dim<std::vector<scalar>> _ref_zmp;   // reference zmp
  xy_dim<std::vector<scalar>> _zmp;   // zmp
  xy_dim<std::vector<scalar>> _input; // input
  xy_dim<scalar> _input_tmp{ 0,0 }; //use to optimatize the calculation of inputs

  // com and last left right joint trajectory
  struct
  {
    std::vector<scalar> com;
    std::vector<scalar> left;
    std::vector<scalar> right;
  }_trajectory;

  //space state com state
  struct
  {
    std::vector<Eigen::Vector3<scalar>> x; // x, vx, ax
    std::vector<Eigen::Vector3<scalar>> y; // y, vy, ay
  }_state;

};
