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
  const std::vector<Pose<double>>& com = walk1.GetTrajectoryCom();
  const std::vector<Pose<double>>& left = walk1.GetTrajectoryLeftLeg();
  const std::vector<Pose<double>>& right = walk1.GetTrajectoryRightLeg();
  const std::vector<double>& ref_zmp_x = walk1.GetRefZmp<1>();
  const std::vector<double>& ref_zmp_y = walk1.GetRefZmp<0>();

  std::ofstream fout("../MatLab/walkpattern.txt", std::ios::out);
  //print com x,y,z
  for (size_t i = 0;i < com.size();++i) {
    fout << com[i].position()(0) << " ";
  }
  fout << std::endl;
  for (size_t i = 0;i < com.size();++i) {
    fout << com[i].position()(1) << " ";
  }
  fout << std::endl;
  for (size_t i = 0;i < com.size();++i) {
    fout << com[i].position()(2) << " ";
  }
  fout << std::endl;

  //print left x,y,z
  for (size_t i = 0;i < left.size();++i) {
    fout << left[i].position()(0) << " ";
  }
  fout << std::endl;
  for (size_t i = 0;i < left.size();++i) {
    fout << left[i].position()(1) << " ";
  }
  fout << std::endl;
  for (size_t i = 0;i < left.size();++i) {
    fout << left[i].position()(2) << " ";
  }
  fout << std::endl;

  //print right x,y,z
  for (size_t i = 0;i < right.size();++i) {
    fout << right[i].position()(0) << " ";
  }
  fout << std::endl;
  for (size_t i = 0;i < right.size();++i) {
    fout << right[i].position()(1) << " ";
  }
  fout << std::endl;
  for (size_t i = 0;i < right.size();++i) {
    fout << right[i].position()(2) << " ";
  }
  fout << std::endl;

  //print ref zmp x,y
  for (size_t i = 0;i < ref_zmp_x.size();++i) {
    fout << ref_zmp_x[i] << " ";
  }
  fout << std::endl;

  for (size_t i = 0;i < ref_zmp_y.size();++i) {
    fout << ref_zmp_y[i] << " ";
  }
  fout << std::endl;

  fout.close();
}
