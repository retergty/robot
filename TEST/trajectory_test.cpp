#include <gtest/gtest.h>
#include "WalkPatternGen.hpp"

class WalkTest : public testing::Test {
protected:
  void SetUp() override {
    walk1.GenerateAStep();
    walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);

    walk2.GenerateContinuousStep(sx, sy, WalkPatternGen<double>::LEG::RIGHT);
    walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);

    walk1.UpdateState();
    walk2.UpdateState();

    walk1.GenerateTrajectoryPosition();
    walk2.GenerateTrajectoryPosition();
  }
  WalkPatternGen<double> walk1;
  WalkPatternGen<double> walk2;
  std::vector<double> sx = { 0,0.2,0.2,0.2,0 };
  std::vector<double> sy = { 0.1,0.2,0.2,0.2,0.1 };
};
TEST_F(WalkTest, walk1_trajectory_test) {
  const std::vector<Pose<double>>& com = walk1.GetTrajectoryCom();
  const std::vector<Pose<double>>& left = walk1.GetTrajectoryLeftLeg();
  const std::vector<Pose<double>>& right = walk1.GetTrajectoryRightLeg();
  const std::vector<WalkPatternGen<double>::LEG> support = walk1.GetTrajectorySupportLeg();
  const std::vector<Eigen::Vector3<double>>& state_x = walk1.GetState<1>();
  const std::vector<Eigen::Vector3<double>>& state_y = walk1.GetState<0>();
  for (size_t i = 0;i < com.size();++i) {
    EXPECT_TRUE(isEqual(com[i].position()(0), state_x[i](0)));
    EXPECT_TRUE(isEqual(com[i].position()(1), state_y[i](0)));
  }
}