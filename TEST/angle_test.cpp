#include <gtest/gtest.h>
#include "WalkPatternGen.hpp"

class AngleTest : public testing::Test {
protected:
  void SetUp() override {
    walk1.GenerateAStep();
    walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);

    walk2.GenerateContinuousStep(sx, sy, WalkPatternGen<double>::LEG::RIGHT);
    walk2.GenerateStillStep(WalkPatternGen<double>::Tstep);

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

TEST_F(AngleTest, walk1_angle_test) {
  std::vector<Eigen::Vector<double, 12>> angle = walk1.GenerateWalkAngle();
}