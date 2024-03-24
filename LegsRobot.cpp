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
}
