#pragma once
#include <vector>
#include "Dense"

// t=0,t=T/2,t=T,t=0,t=T
template <typename scalar>
Eigen::Vector<scalar, 5> FourPolyCurveParm(const scalar T, const Eigen::Vector<scalar, 5>& b)
{
  Eigen::Matrix<scalar, 5, 5> A{
      {1, 0, 0, 0, 0},
      {1, T / 2, T * T / 4, T * T * T / 8, T * T * T * T / 16},
      {1, T, T * T, T * T * T, T * T * T * T},
      {0, 1, 0, 0, 0},
      {0, 1, 2 * T, 3 * T * T, 4 * T * T * T} };
  return A.colPivHouseholderQr().solve(b);
}

template <typename scalar, typename time>
inline scalar FourPolyCurve(const Eigen::Vector<scalar, 5> a, const time t)
{
  return a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t;
}

template <typename scalar>
class WalkPatternGen;

// swing leg trajectory position generation function, use in function GenerateSingleFootSupportTrajectoryPosition
// generate four order polynomial trajectory
template <typename scalar>
void FourOrderPolynomial(
  typename std::vector<scalar>::difference_type zmp_start_index,
  typename std::vector<scalar>::difference_type zmp_end_index,
  std::vector<Pose<scalar>>& swing_leg,
  const xy_dim<scalar>& target_foot_place)
{
  Eigen::Vector3<scalar> swing_leg_position = swing_leg.back().position();
  Eigen::Matrix<scalar, 3, 3> swing_leg_rotation = swing_leg.back().rotation();

  Eigen::Vector<scalar, 5> bx = { swing_leg_position(0), (target_foot_place.x + swing_leg_position(0)) / 2, target_foot_place.x, 0, 0 };
  Eigen::Vector<scalar, 5> by = { swing_leg_position(1), (target_foot_place.y + swing_leg_position(1)) / 2, target_foot_place.y, 0, 0 };
  Eigen::Vector<scalar, 5> bz = { swing_leg_position(2), (target_foot_place.x - swing_leg_position(0)) / 3 + swing_leg_position(2), swing_leg_position(2), 0, 0 };

  const scalar Swing_leg_land_index = zmp_end_index - WalkPatternGen<scalar>::SwingLegPreserveIndex;
  const scalar DiscT = Swing_leg_land_index - zmp_start_index + 1;

  Eigen::Vector<scalar, 5> ax = FourPolyCurveParm(DiscT, bx);
  Eigen::Vector<scalar, 5> ay = FourPolyCurveParm(DiscT, by);
  Eigen::Vector<scalar, 5> az = FourPolyCurveParm(DiscT, bz);

  for (auto i = zmp_start_index; i <= zmp_end_index; ++i)
  {
    if (i <= Swing_leg_land_index)
    {
      size_t curve_index = i - zmp_start_index;
      swing_leg_position = { FourPolyCurve(ax, curve_index), FourPolyCurve(ay, curve_index), FourPolyCurve(az, curve_index) };
    }
    swing_leg.emplace_back(swing_leg_position, swing_leg_rotation);
  }
}

template <typename scalar, typename time>
inline scalar SixPolyCurve(const Eigen::Vector<scalar, 7> a, const time t)
{
  return a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t + a(5) * t * t * t * t * t + a(6) * t * t * t * t * t * t;
}

// important! use long double!
//  t=0,t=T/2,t=T, velocity t=0,t=T, accel t=0, t=T
template <typename scalar>
Eigen::Vector<scalar, 7> SixPolyCurveParm(const scalar T, const Eigen::Vector<scalar, 7>& b)
{
  double T_Ord2 = IntPower<2>(T);
  double T_Ord3 = IntPower<3>(T);
  double T_Ord4 = IntPower<4>(T);
  double T_Ord5 = IntPower<5>(T);
  double T_Ord6 = IntPower<6>(T);

  Eigen::Matrix<scalar, 7, 7> A{
      {1, 0, 0, 0, 0, 0, 0},
      {1, T / 2, T_Ord2 / 4, T_Ord3 / 8, T_Ord4 / 16, T_Ord5 / 32, T_Ord6 / 64},
      {1, T, T_Ord2, T_Ord3, T_Ord4, T_Ord5, T_Ord6},
      {0, 1, 0, 0, 0, 0, 0},
      {0, 1, 2 * T, 3 * T_Ord2, 4 * T_Ord3, 5 * T_Ord4, 6 * T_Ord5},
      {0, 0, 2, 0, 0, 0, 0},
      {0, 0, 2, 6 * T, 12 * T_Ord2, 20 * T_Ord3, 30 * T_Ord4} };
  return A.colPivHouseholderQr().solve(b);
}

template <typename scalar, typename calscalar = scalar>
void SixOrderPolynomial(
  typename std::vector<scalar>::difference_type zmp_start_index,
  typename std::vector<scalar>::difference_type zmp_end_index,
  std::vector<Pose<scalar>>& swing_leg,
  const xy_dim<scalar>& target_foot_place)
{
  static_assert(std::is_same_v<calscalar, long double>);
  Eigen::Vector3<scalar> swing_leg_position = swing_leg.back().position();
  Eigen::Matrix<scalar, 3, 3> swing_leg_rotation = swing_leg.back().rotation();

  Eigen::Vector<scalar, 7> bx = { swing_leg_position(0), (target_foot_place.x + swing_leg_position(0)) / 2, target_foot_place.x, 0, 0, 0, 0 };
  Eigen::Vector<scalar, 7> by = { swing_leg_position(1), (target_foot_place.y + swing_leg_position(1)) / 2, target_foot_place.y, 0, 0, 0, 0 };
  Eigen::Vector<scalar, 7> bz = { swing_leg_position(2), (target_foot_place.x - swing_leg_position(0)) / 3 + swing_leg_position(2), swing_leg_position(2), 0, 0, 0, 0 };

  const scalar Swing_leg_land_index = zmp_end_index - WalkPatternGen<scalar>::SwingLegPreserveIndex;
  const scalar DiscT = Swing_leg_land_index - zmp_start_index + 1;
  Eigen::Vector<calscalar, 7> ax = SixPolyCurveParm<calscalar>(DiscT, bx.template cast<calscalar>());
  Eigen::Vector<calscalar, 7> ay = SixPolyCurveParm<calscalar>(DiscT, by.template cast<calscalar>());
  Eigen::Vector<calscalar, 7> az = SixPolyCurveParm<calscalar>(DiscT, bz.template cast<calscalar>());

  for (auto i = zmp_start_index; i <= zmp_end_index; ++i)
  {
    if (i <= Swing_leg_land_index)
    {
      size_t curve_index = i - zmp_start_index;
      swing_leg_position = { static_cast<scalar>(SixPolyCurve(ax, curve_index)), static_cast<scalar>(SixPolyCurve(ay, curve_index)), static_cast<scalar>(SixPolyCurve(az, curve_index)) };
    }
    swing_leg.emplace_back(swing_leg_position, swing_leg_rotation);
  }
}

template <typename scalar, size_t dimens>
class BaseMethod
{
public:
  BaseMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_total) : _initial(init), _target(target), _time_total(time_total) {};

  virtual ~BaseMethod() {};
  virtual Eigen::Vector<scalar, dimens> NowVal(const scalar now_time) = 0;
protected:
  Eigen::Vector<scalar, dimens> _initial;
  Eigen::Vector<scalar, dimens> _target;
  scalar _time_total;
};


// t=0,t=T/2,t=T,t=0,t=T
template <typename scalar, size_t dimens, typename calscalar = scalar>
class FourPolyMethod final : public BaseMethod<scalar, dimens>
{
public:
  FourPolyMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_total) : BaseMethod<scalar, dimens>(init, target, time_total)
  {
    calscalar T = static_cast<calscalar>(time_total);
    Eigen::Vector<scalar, 5> b[dimens];
    for (size_t i = 0;i < dimens;++i) {
      b[i](0) = init(i);
      b[i](1) = (target(i) + init(i)) / 2;
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

  Eigen::Vector<scalar, dimens> NowVal(const scalar now_time) override
  {
    Eigen::Vector<scalar, dimens> ret;
    calscalar t = static_cast<calscalar>(now_time);
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
class FivePolyMethod final :public BaseMethod<scalar, dimens>
{
public:
  FivePolyMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_total) : BaseMethod<scalar, dimens>(init, target, time_total)
  {
    calscalar T = static_cast<calscalar>(time_total);
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
        {1, T / 2, T_Ord2 / 4, T_Ord3 / 8, T_Ord4 / 16, T_Ord5 / 32},
        {1, T, T_Ord2, T_Ord3, T_Ord4, T_Ord5},
        {0, 1, 0, 0, 0, 0},
        {0, 1, 2 * T, 3 * T_Ord2, 4 * T_Ord3, 5 * T_Ord4},
        {0, 0, 2, 0, 0, 0},
        {0, 0, 2, 6 * T, 12 * T_Ord2, 20 * T_Ord3} };

    for (size_t i = 0;i < dimens;++i) {
      a[i] = A.colPivHouseholderQr().solve(b[i].template cast<calscalar>());
    }
  }

  Eigen::Vector<scalar, dimens> NowVal(const scalar now) override {
    Eigen::Vector<scalar, dimens> ret;
    const calscalar t = static_cast<calscalar>(now);
    for (size_t i = 0;i < dimens;++i) {
      ret(i) = static_cast<scalar>(a[i](0) + a[i](1) * t + a[i](2) * t * t + a[i](3) * t * t * t + a[i](4) * t * t * t * t + a[i](5) * t * t * t * t * t);
    }
  }
private:
  Eigen::Vector<calscalar, 6> a[dimens];
};


// important! use long double!
//  t=0,t=T/2,t=T, velocity t=0,t=T, accel t=0, t=T
template <typename scalar, size_t dimens, typename calscalar = scalar>
class SixPolyMethod final :public BaseMethod<scalar, dimens>
{
public:
  SixPolyMethod(
    const Eigen::Vector<scalar, dimens>& init,
    const Eigen::Vector<scalar, dimens>& target,
    scalar time_total) : BaseMethod<scalar, dimens>(init, target, time_total)
  {
    calscalar T = static_cast<calscalar>(time_total);
    calscalar T_Ord2 = IntPower<2>(T);
    calscalar T_Ord3 = IntPower<3>(T);
    calscalar T_Ord4 = IntPower<4>(T);
    calscalar T_Ord5 = IntPower<5>(T);
    calscalar T_Ord6 = IntPower<6>(T);

    Eigen::Vector<scalar, 7> b[dimens];
    for (size_t i = 0;i < dimens;++i) {
      b[i](0) = init(i);
      b[i](1) = (target(i) + init(i)) / 2;
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

  Eigen::Vector<scalar, dimens> NowVal(const scalar now) override {
    Eigen::Vector<scalar, dimens> ret;
    calscalar t = static_cast<calscalar>(now);
    for (size_t i = 0;i < dimens;++i) {
      ret(i) = static_cast<scalar>(a[i](0) + a[i](1) * t + a[i](2) * t * t + a[i](3) * t * t * t + a[i](4) * t * t * t * t + a[i](5) * t * t * t * t * t + a[i](6) * t * t * t * t * t * t);
    }
    return ret;
  }
private:
  Eigen::Vector<calscalar, 7> a[dimens];
};