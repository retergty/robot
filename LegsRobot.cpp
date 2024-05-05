#include "LegsRobot.hpp"
#include "WalkPatternGen.hpp"
#include <iostream>
#include <fstream>
template<typename scalar>
void GenerateWalkPatternToFile(const WalkPatternGen<scalar>& walk, std::ofstream& f);
int main()
{
  WalkPatternGen<double> walk1;
  walk1.GenerateAStep();
  walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);
  walk1.UpdateState();
  walk1.GenerateTrajectoryPosition();
  std::ofstream fout1("../MatLab/walkpattern1.txt", std::ios::out);
  GenerateWalkPatternToFile(walk1, fout1);

  WalkPatternGen<double> walk2;
  std::vector<double> sx = { 0,param::STEP_LENGTH,param::STEP_LENGTH,param::STEP_LENGTH,0 };
  std::vector<double> sy = { param::STEP_WIDTH / 2,param::STEP_WIDTH,param::STEP_WIDTH,param::STEP_WIDTH,param::STEP_WIDTH / 2 };
  std::vector<double> sz = {0,0,0,0,0};
  walk2.GenerateContinuousStep(sx, sy, sz,WalkPatternGen<double>::LEG::RIGHT);
  walk2.GenerateStillStep(WalkPatternGen<double>::Tstep);
  walk2.UpdateState();
  walk2.GenerateTrajectoryPosition();
  std::ofstream fout2("../MatLab/walkpattern2.txt", std::ios::out);
  GenerateWalkPatternToFile(walk2, fout2);

  //upstairs 
  WalkPatternGen<double> walk3;
  walk3.GenerateAStep(param::STEP_LENGTH,param::STEP_WIDTH,0.02);
  walk3.GenerateStillStep(WalkPatternGen<double>::Tstep);
  walk3.UpdateState();
  walk3.GenerateTrajectoryPosition<StairsMethod<double,long double>,FivePolyMethod<double,1,long double>>();
  std::ofstream fout3("../MatLab/walkpattern3.txt", std::ios::out);
  GenerateWalkPatternToFile(walk3, fout3);

  //upstairs 
  WalkPatternGen<double> walk4;
  walk4.GenerateAStep(param::STEP_LENGTH,param::STEP_WIDTH,-0.02);
  walk4.GenerateStillStep(WalkPatternGen<double>::Tstep);
  walk4.UpdateState();
  walk4.GenerateTrajectoryPosition<StairsMethod<double,long double>,FivePolyMethod<double,1,long double>>();
  std::ofstream fout4("../MatLab/walkpattern4.txt", std::ios::out);
  GenerateWalkPatternToFile(walk4, fout4);

}
template<typename scalar>
void GenerateWalkPatternToFile(const WalkPatternGen<scalar>& walk, std::ofstream& fout) {
  const std::vector<Pose<scalar>>& com = walk.GetTrajectoryCom();
  const std::vector<Pose<scalar>>& left = walk.GetTrajectoryLeftLeg();
  const std::vector<Pose<scalar>>& right = walk.GetTrajectoryRightLeg();
  const std::vector<scalar>& ref_zmp_x = walk.template GetRefZmp<1>();
  const std::vector<scalar>& ref_zmp_y = walk.template GetRefZmp<0>();
  const std::vector<scalar>& zmp_x = walk.template GetZmp<1>();
  const std::vector<scalar>& zmp_y = walk.template GetZmp<0>();

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

  //print zmp x,y
  for (size_t i = 0;i < zmp_x.size();++i) {
    fout << zmp_x[i] << " ";
  }
  fout << std::endl;

  for (size_t i = 0;i < zmp_y.size();++i) {
    fout << zmp_y[i] << " ";
  }
  fout << std::endl;

  fout.close();
}