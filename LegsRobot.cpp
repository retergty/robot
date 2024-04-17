#include "LegsRobot.hpp"
#include "WalkPatternGen.hpp"
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

  Eigen::Matrix3d m1 = Eigen::AngleAxis<double>(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::cout << m1 << std::endl;

  std::cout << m1 * Eigen::Vector3d::UnitX() << std::endl;
  WalkPatternGen<double> walk;

}
