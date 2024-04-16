#include "Dense"
#include "Geometry"
#include "LegsRobot.hpp"
#include <iostream>
#include <fstream>

template<typename scalar>
class WalkPatternGen
{
public:

private:
  // simulation parmeter
  scalar _sample_time;
  scalar _Zc;
  Eigen::Matrixx<scalar> _A;
  Eigen::Matrixx<scalar> _B;
  Eigen::Matrixx<scalar> _C;
  size_t _prv_num;
  std::vector<scalar> _G;

  LegsRobot<scalar> _legrobot;

  

};
