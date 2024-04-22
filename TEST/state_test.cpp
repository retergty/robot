#include <gtest/gtest.h>
#include "WalkPatternGen.hpp"
#include "preview_test1.hpp"
#include "preview_test2.hpp"

class WalkTest : public testing::Test {
protected:
  void SetUp() override {
    walk1.GenerateAStep();
    walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);

    walk2.GenerateContinuousStep(sx, sy, WalkPatternGen<double>::LEG::RIGHT);
    walk1.GenerateStillStep(WalkPatternGen<double>::Tstep);

    walk1.UpdateState();
    walk2.UpdateState();
  }
  WalkPatternGen<double> walk1;
  WalkPatternGen<double> walk2;
  std::vector<double> sx = { 0,0.2,0.2,0.2,0 };
  std::vector<double> sy = { 0.1,0.2,0.2,0.2,0.1 };
};

TEST_F(WalkTest, walk1_reference_zmp_test) {
  const std::vector<double>& ref_x = walk1.GetRefZmp<1>();
  const std::vector<double>& ref_y = walk1.GetRefZmp<0>();
  for (size_t i = 0;i < ref_x.size();++i) {
    EXPECT_TRUE(isEqual(ref_x[i], TEST1_REFERENCE_ZMPX[i]));
  }
  for (size_t i = 0;i < ref_y.size();++i) {
    EXPECT_TRUE(isEqual(ref_y[i], TEST1_REFERENCE_ZMPY[i]));
  }
}
TEST_F(WalkTest, walk2_reference_zmp_test) {
  const std::vector<double>& ref_x = walk2.GetRefZmp<1>();
  const std::vector<double>& ref_y = walk2.GetRefZmp<0>();
  for (size_t i = 0;i < ref_x.size();++i) {
    EXPECT_TRUE(isEqual(ref_x[i], TEST2_REFERENCE_ZMPX[i]));
  }
  for (size_t i = 0;i < ref_y.size();++i) {
    EXPECT_TRUE(isEqual(ref_y[i], TEST2_REFERENCE_ZMPY[i]));
  }
}

TEST_F(WalkTest, walk1_zmp_test) {
  const std::vector<double>& zmp_x = walk1.GetZmp<1>();
  const std::vector<double>& zmp_y = walk1.GetZmp<0>();
  for (size_t i = 0;i < zmp_x.size();++i) {
    EXPECT_TRUE(isEqual(zmp_x[i], TEST1_REFERENCE_STATEZMPX[i]));
  }
  for (size_t i = 0;i < zmp_y.size();++i) {
    EXPECT_TRUE(isEqual(zmp_y[i], TEST1_REFERENCE_STATEZMPY[i]));
  }
}
TEST_F(WalkTest, walk2_zmp_test) {
  const std::vector<double>& zmp_x = walk2.GetZmp<1>();
  const std::vector<double>& zmp_y = walk2.GetZmp<0>();
  for (size_t i = 0;i < zmp_x.size();++i) {
    EXPECT_TRUE(isEqual(zmp_x[i], TEST2_REFERENCE_STATEZMPX[i]));
  }
  for (size_t i = 0;i < zmp_y.size();++i) {
    EXPECT_TRUE(isEqual(zmp_y[i], TEST2_REFERENCE_STATEZMPY[i]));
  }
}

TEST_F(WalkTest, walk1_state_test) {
  const std::vector<Eigen::Vector3<double>>& state_x = walk1.GetState<1>();
  const std::vector<Eigen::Vector3<double>>& state_y = walk1.GetState<0>();

  for (size_t i = 0;i < state_x.size();++i) {
    EXPECT_TRUE(isEqual(state_x[i](0), TEST1_REFERENCE_STATEX1[i]));
    EXPECT_TRUE(isEqual(state_x[i](1), TEST1_REFERENCE_STATEX2[i]));
    EXPECT_TRUE(isEqual(state_x[i](2), TEST1_REFERENCE_STATEX3[i]));
  }

  for (size_t i = 0;i < state_y.size();++i) {
    EXPECT_TRUE(isEqual(state_y[i](0), TEST1_REFERENCE_STATEY1[i]));
    EXPECT_TRUE(isEqual(state_y[i](1), TEST1_REFERENCE_STATEY2[i]));
    EXPECT_TRUE(isEqual(state_y[i](2), TEST1_REFERENCE_STATEY3[i]));
  }
}
TEST_F(WalkTest, walk2_state_test) {
  const std::vector<Eigen::Vector3<double>>& state_x = walk2.GetState<1>();
  const std::vector<Eigen::Vector3<double>>& state_y = walk2.GetState<0>();

  for (size_t i = 0;i < state_x.size();++i) {
    EXPECT_TRUE(isEqual(state_x[i](0), TEST2_REFERENCE_STATEX1[i]));
    EXPECT_TRUE(isEqual(state_x[i](1), TEST2_REFERENCE_STATEX2[i]));
    EXPECT_TRUE(isEqual(state_x[i](2), TEST2_REFERENCE_STATEX3[i]));
  }

  for (size_t i = 0;i < state_y.size();++i) {
    EXPECT_TRUE(isEqual(state_y[i](0), TEST2_REFERENCE_STATEY1[i]));
    EXPECT_TRUE(isEqual(state_y[i](1), TEST2_REFERENCE_STATEY2[i]));
    EXPECT_TRUE(isEqual(state_y[i](2), TEST2_REFERENCE_STATEY3[i]));
  }
}