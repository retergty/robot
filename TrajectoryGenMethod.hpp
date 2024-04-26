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
  Eigen::Matrix<scalar,3,3> swing_leg_rotation = swing_leg.back().rotation();

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
