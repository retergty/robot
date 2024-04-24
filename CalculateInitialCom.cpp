#include "LegsRobot.hpp"
#include "WalkPatternGen.hpp"
#include <iostream>
#include <fstream>
int main()
{
  LegsRobot<double> legrobot({0,0,param::COM_Z - 0.018});
  //legrobot.Forward_kinematics(angle);
  std::cout << legrobot.massCenter() << std::endl;
  std::cout << legrobot.legCenter() << std::endl;
  std::cout << legrobot.rightLeg().back() << std::endl;
  std::cout << legrobot.leftLeg().back() << std::endl;
}