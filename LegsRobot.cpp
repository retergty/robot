#include "LegsRobot.hpp"
#include <iostream>
int main()
{
  std::cout << "Hello World!" << std::endl;
  Eigen::Matrix4d m;
  for (size_t i = 0, nums = 0;i < 4;++i) {
    for (size_t j = 0;j < 4;++j) {
      m(i, j) = ++nums;
    }
  }

  std::cout << m << std::endl << m.topLeftCorner<1, 3>() << std::endl;;
  std::cout << m.topRightCorner<2, 3>() << std::endl;

  Eigen::Matrix3d m1 = Eigen::Matrix3d::Identity();
  Eigen::AngleAxis<double> angax(0.25 * M_PI, Eigen::Vector3d::UnitX());

  Joint<double> j(Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX(), 0);
  Eigen::Vector3d::Zero();

  LegsRobot<double> legr;
}
