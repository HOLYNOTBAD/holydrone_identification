#include <gtest/gtest.h>
#include <stdexcept>
#include <holydrone_identification/recenter_control.hpp>
#include <holydrone_identification/excitation_plan.hpp>
TEST(Recenter, DirectionQuaternionSignAndBound) {
  identification::RecenterControl c;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  EXPECT_LT(c.rates(q, zero, Eigen::Vector3d(1,0,0), 0).y(), 0); // East velocity -> negative pitch
  EXPECT_GT(c.rates(q, zero, Eigen::Vector3d(0,1,0), 0).x(), 0); // North velocity -> positive roll
  EXPECT_GT(c.rates(q, Eigen::Vector3d(1,0,0), zero, 0).y(), 0);
  q = Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitX());
  const auto rate = c.rates(q, zero, zero, 0);
  EXPECT_LT(rate.x(), 0);
  q.coeffs() *= -1;
  EXPECT_LT((c.rates(q,zero,zero,0)-rate).norm(), 1e-10);
  EXPECT_LE(c.rates(q,Eigen::Vector3d(100,100,0),zero,3).norm(), c.max_rate+1e-10);
}
TEST(Recenter, YawedBodyStillBrakesEast) {
  identification::RecenterControl c;
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));
  auto r = c.rates(q, Eigen::Vector3d::Zero(), Eigen::Vector3d(1,0,0), M_PI/2);
  EXPECT_LT(r.x(), 0); EXPECT_NEAR(r.y(),0,1e-10); EXPECT_NEAR(r.z(),0,1e-10);
}
TEST(Plans, BalancedPairsAndPreservedCoupledSigns) {
  const std::array<double,3> rates{{.2,.3,.25}};
  auto first = identification::firstOrderPlan(rates,.015,3);
  ASSERT_EQ(first.size(),12u);
  for (const auto& g : first) {
    ASSERT_EQ(g.pulses.size(),2u);
    for (int i=0;i<4;++i) EXPECT_DOUBLE_EQ(g.pulses[0][i],-g.pulses[1][i]);
  }
  auto physics = identification::physicsPlan(rates,.025,2);
  ASSERT_EQ(physics.size(),24u);
  int count=0;
  for (const auto& g : physics) {
    count+=g.pulses.size();
    for (int i=0;i<4;++i) {
      double sum=0; for(const auto& p:g.pulses) sum+=p[i]; EXPECT_NEAR(sum,0,1e-10);
    }
    if (g.channel=="coupled") {
      ASSERT_EQ(g.pulses.size(),2u);
      for (int axis=0;axis<3;++axis) EXPECT_DOUBLE_EQ(g.pulses[0][axis],-g.pulses[1][axis]);
    }
  }
  EXPECT_EQ(count,48);
  EXPECT_GT(physics[9].pulses[0][0],0); EXPECT_LT(physics[9].pulses[0][1],0);
  EXPECT_LT(physics[10].pulses[0][0],0); EXPECT_GT(physics[10].pulses[0][1],0);
}
TEST(Plans, MultipleWidthsAndTwoAxisCoupling) {
  const auto plan=identification::physicsPlan({{.3,.3,.25}},.025,2,{.2,.35,.5});
  ASSERT_EQ(plan.size(),44u);
  int two_axis=0;
  for(const auto& g:plan) {
    EXPECT_GE(g.hold_s,.2); EXPECT_LE(g.hold_s,.5);
    ASSERT_EQ(g.pulses.size(),2u);
    int active=0;
    for(int i=0;i<4;++i) {
      EXPECT_DOUBLE_EQ(g.pulses[0][i],-g.pulses[1][i]);
      if(i<3 && g.pulses[0][i]!=0) ++active;
    }
    if(active==2) ++two_axis;
  }
  EXPECT_EQ(two_axis,12);
}
TEST(Plans, YawSweepIntegerCyclesAndRecoverySegments) {
  const auto base=identification::physicsPlan({{.3,.3,.25}},.025,2,{.2,.35,.5});
  const auto plan=identification::withYawSweep(base,{.5,1.,2.,3.},4,.25);
  ASSERT_EQ(plan.size(),46u);
  int swept=0;
  for(const auto& g:plan) {
    if(g.channel!="yaw") { EXPECT_EQ(g.sine_hz,0); continue; }
    ++swept; ASSERT_EQ(g.pulses.size(),1u);
    EXPECT_DOUBLE_EQ(g.sine_hz*g.hold_s,4.);
    EXPECT_DOUBLE_EQ(identification::waveformScale(g,0),0);
    EXPECT_DOUBLE_EQ(identification::waveformScale(g,g.hold_s),0);
    EXPECT_NEAR(identification::waveformScale(g,.25/g.sine_hz),1,1e-12);
    EXPECT_NEAR(identification::waveformScale(g,.75/g.sine_hz),-1,1e-12);
    double integral=0;
    for(int i=0;i<1000;++i) integral+=identification::waveformScale(g,g.hold_s*(i+.5)/1000);
    EXPECT_NEAR(integral,0,1e-9);
  }
  EXPECT_EQ(swept,8);
  EXPECT_THROW(identification::withYawSweep(base,{0.},4,.25),std::runtime_error);
  EXPECT_THROW(identification::withYawSweep(base,{1.},0,.25),std::runtime_error);
}
TEST(Plans, SecondOrderSingleAxisMultipleDurations) {
  const auto plan=identification::secondOrderPlan({{.2,.2,.25}},.015,2,{.2,.35,.5});
  ASSERT_EQ(plan.size(),24u);
  for (const auto& g : plan) {
    ASSERT_EQ(g.pulses.size(),2u);
    int active=0;
    for (int i=0;i<4;++i) {
      EXPECT_DOUBLE_EQ(g.pulses[0][i],-g.pulses[1][i]);
      if (g.pulses[0][i]!=0) ++active;
    }
    EXPECT_EQ(active,1);
    EXPECT_TRUE(g.hold_s==.2 || g.hold_s==.35 || g.hold_s==.5);
    EXPECT_TRUE(g.repetition==0 || g.repetition==1);
  }
  EXPECT_THROW(identification::secondOrderPlan({{.2,.2,.25}},.015,2,{}),std::runtime_error);
  EXPECT_THROW(identification::secondOrderPlan({{.2,.2,.25}},.015,2,{.01}),std::runtime_error);
}
int main(int argc,char**argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
