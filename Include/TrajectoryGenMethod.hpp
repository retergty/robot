#pragma once
#include <vector>
#include "Dense"
#include "RobotParamDef.hpp"

template <typename scalar, size_t dimens>
class BaseMethod
{
public:
  BaseMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_start,
    scalar time_end) : _initial(init), _target(target), _time_start(time_start), _time_end(time_end) {};

  virtual ~BaseMethod() {};
  virtual Eigen::Vector<scalar, dimens> NowVal(const scalar now_time) const = 0;
protected:
  Eigen::Vector<scalar, dimens> _initial;
  Eigen::Vector<scalar, dimens> _target;
  scalar _time_start;
  scalar _time_end;
};


// t=0,t=T/2,t=T,t=0,t=T
template <typename scalar, size_t dimens, typename calscalar = scalar, bool peek_z = true>
class FourPolyMethod : public BaseMethod<scalar, dimens>
{
public:
  using value_type = scalar;
  using cal_value_type = calscalar;
  FourPolyMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_start,
    scalar time_end) : BaseMethod<scalar, dimens>(init, target, time_start, time_end)
  {
    calscalar T = static_cast<calscalar>(time_end - time_start + 1);
    Eigen::Vector<scalar, 5> b[dimens];
    for (size_t i = 0;i < dimens;++i) {
      b[i](0) = init(i);
      if constexpr (peek_z) {
        if (i == 2) {
          b[i](1) = init(i) + param::STEP_Z_PEEK; // if z axis init is equal to target, generate a peek to prevent line trajectory
        }
        else {
          b[i](1) = (target(i) + init(i)) / 2;
        }
      }
      else {
        b[i](1) = (target(i) + init(i)) / 2;
      }
      b[i](2) = target(i);
      b[i](3) = 0;
      b[i](4) = 0;
    }
    Eigen::Matrix<calscalar, 5, 5> A{
        {1, 0, 0, 0, 0},
        {1, T / 2, T * T / 4, T * T * T / 8, T * T * T * T / 16},
        {1, T, T * T, T * T * T, T * T * T * T},
        {0, 1, 0, 0, 0},
        {0, 1, 2 * T, 3 * T * T, 4 * T * T * T} };
    for (size_t i = 0; i < dimens; ++i)
    {
      a[i] = A.colPivHouseholderQr().solve(b[i].template cast<calscalar>());
    }
  };

  Eigen::Vector<scalar, dimens> NowVal(const scalar now) const override
  {
    if (now < this->_time_start) {
      return this->_initial;
    }
    else if (now > this->_time_end) {
      return this->_target;
    }

    Eigen::Vector<scalar, dimens> ret;
    const calscalar t = static_cast<calscalar>(now - this->_time_start);
    for (size_t i = 0;i < dimens;++i) {
      ret(i) = static_cast<scalar>(a[i](0) + a[i](1) * t + a[i](2) * t * t + a[i](3) * t * t * t + a[i](4) * t * t * t * t);
    }
    return ret;
  }

private:
  Eigen::Vector<calscalar, 5> a[dimens];
};

// t=0.t=T, vel t=0,t=T, acc t=0,t=T
template<typename scalar, size_t dimens, typename calscalar = scalar>
class FivePolyMethod :public BaseMethod<scalar, dimens>
{
public:
  using value_type = scalar;
  using cal_value_type = calscalar;
  FivePolyMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_start,
    scalar time_end) : BaseMethod<scalar, dimens>(init, target, time_start, time_end)
  {
    calscalar T = static_cast<calscalar>(time_end - time_start + 1);
    calscalar T_Ord2 = IntPower<2>(T);
    calscalar T_Ord3 = IntPower<3>(T);
    calscalar T_Ord4 = IntPower<4>(T);
    calscalar T_Ord5 = IntPower<5>(T);

    Eigen::Vector<scalar, 6> b[dimens];

    for (size_t i = 0;i < dimens;++i) {
      b[i](0) = init(i);
      b[i](1) = target(i);
      b[i](2) = b[i](3) = 0;
      b[i](4) = b[i](5) = 0;
    }
    Eigen::Matrix<calscalar, 6, 6> A{
        {1, 0, 0, 0, 0, 0},
        {1, T, T_Ord2, T_Ord3, T_Ord4, T_Ord5},
        {0, 1, 0, 0, 0, 0},
        {0, 1, 2 * T, 3 * T_Ord2, 4 * T_Ord3, 5 * T_Ord4},
        {0, 0, 2, 0, 0, 0},
        {0, 0, 2, 6 * T, 12 * T_Ord2, 20 * T_Ord3} };

    for (size_t i = 0;i < dimens;++i) {
      a[i] = A.colPivHouseholderQr().solve(b[i].template cast<calscalar>());
    }
  }

  Eigen::Vector<scalar, dimens> NowVal(const scalar now) const override {
    if (now < this->_time_start) {
      return this->_initial;
    }
    else if (now > this->_time_end) {
      return this->_target;
    }

    Eigen::Vector<scalar, dimens> ret;
    const calscalar t = static_cast<calscalar>(now - this->_time_start);
    for (size_t i = 0;i < dimens;++i) {
      ret(i) = static_cast<scalar>(a[i](0) + a[i](1) * t + a[i](2) * t * t + a[i](3) * t * t * t + a[i](4) * t * t * t * t + a[i](5) * t * t * t * t * t);
    }
    return ret;
  }
private:
  Eigen::Vector<calscalar, 6> a[dimens];
};


// important! use long double!
//  t=0,t=T/2,t=T, velocity t=0,t=T, accel t=0, t=T
template <typename scalar, size_t dimens, typename calscalar = scalar, bool peek_z = true>
class SixPolyMethod :public BaseMethod<scalar, dimens>
{
public:
  using value_type = scalar;
  using cal_value_type = calscalar;
  SixPolyMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_start,
    scalar time_end) : BaseMethod<scalar, dimens>(init, target, time_start, time_end)
  {
    calscalar T = static_cast<calscalar>(time_end - time_start + 1);
    calscalar T_Ord2 = IntPower<2>(T);
    calscalar T_Ord3 = IntPower<3>(T);
    calscalar T_Ord4 = IntPower<4>(T);
    calscalar T_Ord5 = IntPower<5>(T);
    calscalar T_Ord6 = IntPower<6>(T);

    Eigen::Vector<scalar, 7> b[dimens];
    for (size_t i = 0;i < dimens;++i) {
      b[i](0) = init(i);
      if constexpr (peek_z) {
        if (i == 2) {
          b[i](1) = init(i) + param::STEP_Z_PEEK; // if z axis init is equal to target, generate a peek to prevent line trajectory
        }
        else {
          b[i](1) = (target(i) + init(i)) / 2;
        }
      }
      else {
        b[i](1) = (target(i) + init(i)) / 2;
      }
      b[i](2) = target(i);
      b[i](3) = b[i](4) = 0;
      b[i](5) = b[i](6) = 0;
    }

    Eigen::Matrix<calscalar, 7, 7> A{
        {1, 0, 0, 0, 0, 0, 0},
        {1, T / 2, T_Ord2 / 4, T_Ord3 / 8, T_Ord4 / 16, T_Ord5 / 32, T_Ord6 / 64},
        {1, T, T_Ord2, T_Ord3, T_Ord4, T_Ord5, T_Ord6},
        {0, 1, 0, 0, 0, 0, 0},
        {0, 1, 2 * T, 3 * T_Ord2, 4 * T_Ord3, 5 * T_Ord4, 6 * T_Ord5},
        {0, 0, 2, 0, 0, 0, 0},
        {0, 0, 2, 6 * T, 12 * T_Ord2, 20 * T_Ord3, 30 * T_Ord4} };
    for (size_t i = 0;i < dimens;++i) {
      a[i] = A.colPivHouseholderQr().solve(b[i].template cast<calscalar>());
    }
  }

  Eigen::Vector<scalar, dimens> NowVal(const scalar now) const override
  {
    if (now < this->_time_start) {
      return this->_initial;
    }
    else if (now > this->_time_end) {
      return this->_target;
    }

    Eigen::Vector<scalar, dimens> ret;
    const calscalar t = static_cast<calscalar>(now - this->_time_start);
    for (size_t i = 0;i < dimens;++i) {
      ret(i) = static_cast<scalar>(a[i](0) + a[i](1) * t + a[i](2) * t * t + a[i](3) * t * t * t + a[i](4) * t * t * t * t + a[i](5) * t * t * t * t * t + a[i](6) * t * t * t * t * t * t);
    }
    return ret;
  }
private:
  Eigen::Vector<calscalar, 7> a[dimens];
};

template <typename scalar, size_t dimens, typename calscalar = scalar>
class KeepMethod : public BaseMethod<scalar, dimens>
{
public:
  using value_type = scalar;
  using cal_value_type = calscalar;
  KeepMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_start,
    scalar time_end) : BaseMethod<scalar, dimens>(init, target, time_start, time_end)
  {
    for (size_t i = 0;i < dimens;++i) {
      assert(isEqual(init(i), target(i)));
    }
  };
  Eigen::Vector<scalar, dimens> NowVal(const scalar now) const override {
    (void)now;
    return this->_initial;
  }
};

// swing leg cross stairs method
template<typename scalar, typename calscalar = scalar>
class StairsMethod : public FivePolyMethod<scalar, 3, calscalar>
{
public:
  using value_type = scalar;
  using cal_value_type = calscalar;
  StairsMethod(
    const Eigen::Vector<scalar, 3>& init,
    const Eigen::Vector<scalar, 3>& target,
    scalar time_start,
    scalar time_end) : FivePolyMethod<scalar, 3, calscalar>(init, target, time_start, time_end) {
    if (target(2) > init(2)) {
      _com_z_peek = (target(2) - init(2)) * 0.8;
    }
    else {
      _com_z_peek = (init(2) - target(2)) * 0.8;
    }
  }

  Eigen::Vector<scalar, 3> NowVal(const scalar now) const override {

    if (now < this->_time_start) {
      return this->_initial;
    }
    else if (now > this->_time_end) {
      return this->_target;
    }

    Eigen::Vector<scalar, 3>  ret = FivePolyMethod<scalar, 3, calscalar>::NowVal(now);

    const calscalar t = static_cast<calscalar>(now - this->_time_start);
    const calscalar period = static_cast<calscalar>(this->_time_end - this->_time_start + 1);
    ret(0) += -_com_z_peek + _com_z_peek * std::cos(2 * M_PI * t / period);
    ret(2) += _com_z_peek - _com_z_peek * std::cos(2 * M_PI * t / period);
    return ret;
  }
private:
  scalar _com_z_peek;
};