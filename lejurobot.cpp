#include "LegsRobot.hpp"
#include "WalkPatternGen.hpp"
#include <iostream>
#include <fstream>

int main()
{
  WalkPatternGen<double> walk1;
  walk1.GenerateAStep();
  walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);
  walk1.UpdateState();
  walk1.GenerateTrajectoryPosition();
  
  return 0;
}