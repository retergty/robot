#pragma once
#include <vector>
#include "Dense"

// t=0,t=T/2,t=T,t=0,t=T
template<typename scalar>
Eigen::Vector<scalar, 5> FourPolyCurveParm(const scalar T, const Eigen::Vector<scalar, 5>& b) {
  Eigen::Matrix<scalar, 5, 5> A{
    {1,0,0,0,0},
    {1,T / 2,T * T / 4,T * T * T / 8,T * T * T * T / 16},
    {1,T,T * T,T * T * T,T * T * T * T},
    {0,1,0,0,0},
    {0,1,2 * T,3 * T * T,4 * T * T * T}
  };
  return A.colPivHouseholderQr().solve(b);
}

template<typename scalar, typename time>
inline scalar FourPolyCurve(const Eigen::Vector<scalar, 5> a, const time t) {
  return a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t;
}

template<typename scalar>
class WalkPatternGen;

//swing leg trajectory position generation function, use in function GenerateSingleFootSupportTrajectoryPosition
//generate four order polynomial trajectory
template<typename scalar>
void FourOrderPolynomial(
  typename std::vector<scalar>::difference_type zmp_start_index,
  typename std::vector<scalar>::difference_type zmp_end_index,
  std::vector<Pose<scalar>>& swing_leg,
  const xy_dim<scalar>& target_foot_place) {
  Eigen::Vector3<scalar> swing_leg_position = swing_leg.back().position();
  Eigen::Matrix<scalar, 3, 3> swing_leg_rotation = swing_leg.back().rotation();

  Eigen::Vector<scalar, 5> bx = { swing_leg_position(0),(target_foot_place.x + swing_leg_position(0)) / 2,target_foot_place.x,0,0 };
  Eigen::Vector<scalar, 5> by = { swing_leg_position(1),(target_foot_place.y + swing_leg_position(1)) / 2,target_foot_place.y,0,0 };
  Eigen::Vector<scalar, 5> bz = { swing_leg_position(2),(target_foot_place.x - swing_leg_position(0)) / 3 + swing_leg_position(2),swing_leg_position(2),0,0 };

  const scalar Swing_leg_land_index = zmp_end_index - WalkPatternGen<scalar>::SwingLegPreserveIndex;
  const scalar DiscT = Swing_leg_land_index - zmp_start_index + 1;

  Eigen::Vector<scalar, 5> ax = FourPolyCurveParm(DiscT, bx);
  Eigen::Vector<scalar, 5> ay = FourPolyCurveParm(DiscT, by);
  Eigen::Vector<scalar, 5> az = FourPolyCurveParm(DiscT, bz);

  for (auto i = zmp_start_index;i <= zmp_end_index;++i) {
    if (i <= Swing_leg_land_index) {
      size_t curve_index = i - zmp_start_index;
      swing_leg_position = { FourPolyCurve(ax,curve_index),FourPolyCurve(ay,curve_index),FourPolyCurve(az,curve_index) };
    }
    swing_leg.emplace_back(swing_leg_position, swing_leg_rotation);
  }
}

template<typename scalar, typename time>
inline scalar SixPolyCurve(const Eigen::Vector<scalar, 7> a, const time t) {
  return a(0) + a(1) * t + a(2) * t * t + a(3) * t * t * t + a(4) * t * t * t * t + a(5) * t * t * t * t * t + a(6) * t * t * t * t * t * t;
}

//important! use long double!
// t=0,t=T/2,t=T, velocity t=0,t=T, accel t=0, t=T
template<typename scalar>
Eigen::Vector<scalar, 7> SixPolyCurveParm(const scalar T, const Eigen::Vector<scalar, 7>& b) {
  double T_Ord2 = IntPower<2>(T);
  double T_Ord3 = IntPower<3>(T);
  double T_Ord4 = IntPower<4>(T);
  double T_Ord5 = IntPower<5>(T);
  double T_Ord6 = IntPower<6>(T);

  Eigen::Matrix<scalar, 7, 7> A{
    {1,0,0,0,0,0,0},
    {1,T / 2,T_Ord2 / 4,T_Ord3 / 8,T_Ord4 / 16,T_Ord5 / 32,T_Ord6 / 64},
    {1,T,T_Ord2,T_Ord3,T_Ord4,T_Ord5,T_Ord6},
    {0,1,0,0,0,0,0},
    {0,1,2 * T,3 * T_Ord2,4 * T_Ord3,5 * T_Ord4,6 * T_Ord5},
    {0,0,2,0,0,0,0},
    {0,0,2,6 * T,12 * T_Ord2,20 * T_Ord3,30 * T_Ord4}
  };
  return A.colPivHouseholderQr().solve(b);
}

template<typename scalar,typename calscalar = scalar>
void SixOrderPolynomial(
  typename std::vector<scalar>::difference_type zmp_start_index,
  typename std::vector<scalar>::difference_type zmp_end_index,
  std::vector<Pose<scalar>>& swing_leg,
  const xy_dim<scalar>& target_foot_place)
{
  static_assert(std::is_same_v<calscalar, long double>);
  Eigen::Vector3<scalar> swing_leg_position = swing_leg.back().position();
  Eigen::Matrix<scalar, 3, 3> swing_leg_rotation = swing_leg.back().rotation();

  Eigen::Vector<scalar, 7> bx = { swing_leg_position(0),(target_foot_place.x + swing_leg_position(0)) / 2,target_foot_place.x,0,0,0,0 };
  Eigen::Vector<scalar, 7> by = { swing_leg_position(1),(target_foot_place.y + swing_leg_position(1)) / 2,target_foot_place.y,0,0,0,0 };
  Eigen::Vector<scalar, 7> bz = { swing_leg_position(2),(target_foot_place.x - swing_leg_position(0)) / 3 + swing_leg_position(2),swing_leg_position(2),0,0,0,0 };

  const scalar Swing_leg_land_index = zmp_end_index - WalkPatternGen<scalar>::SwingLegPreserveIndex;
  const scalar DiscT = Swing_leg_land_index - zmp_start_index + 1;
  Eigen::Vector<calscalar, 7> ax = SixPolyCurveParm<calscalar>(DiscT, bx.template cast<calscalar>());
  Eigen::Vector<calscalar, 7> ay = SixPolyCurveParm<calscalar>(DiscT, by.template cast<calscalar>());
  Eigen::Vector<calscalar, 7> az = SixPolyCurveParm<calscalar>(DiscT, bz.template cast<calscalar>());

  for (auto i = zmp_start_index;i <= zmp_end_index;++i) {
    if (i <= Swing_leg_land_index) {
      size_t curve_index = i - zmp_start_index;
      swing_leg_position = { static_cast<scalar>(SixPolyCurve(ax,curve_index)),static_cast<scalar>(SixPolyCurve(ay,curve_index)),static_cast<scalar>(SixPolyCurve(az,curve_index)) };
    }
    swing_leg.emplace_back(swing_leg_position, swing_leg_rotation);
  }
}

template<typename scalar,size_t dimen,typename timeType>
class BaseMethod
{
public:
  BaseMethod(const Eigen::Vector<scalar,dimen>& init,const Eigen::Vector<scalar,dimen>& target,timeType begin,timeType end): _initial(init),_target(target),_begin(begin),_end(end){};
  virtual ~BaseMethod(){}:
protect:
  Eigen::Vector<scalar,dimen> _initial;
  Eigen::Vector<scalar,dimen> _target;
  timeType _begin;
  timeType _end;
};

