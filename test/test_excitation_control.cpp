#include <excitation/settling_gate.hpp>
#include <gtest/gtest.h>
#include <stdexcept>
#include <excitation/recenter_control.hpp>
#include <excitation/excitation_plan.hpp>
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
TEST(Recenter, HandoverRemovesResidualTiltInsteadOfFreezingIt) {
  // 平面定高模型：3 度残余俯仰、0.1 m/s 漂移，角速度响应时间常数 0.08 s。
  identification::RecenterControl control;
  for (bool feedback : {false, true}) {
    double pitch = 3*M_PI/180, rate = 0, x = .1, vx = .1;
    identification::SettlingGate gate;
    bool settled = false;
    const double dt = .002;
    for (double t = 0; t < 4; t += dt) {
      Eigen::Quaterniond q(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
      const double command = feedback ? control.rates(q, Eigen::Vector3d(-x,0,0),
          Eigen::Vector3d(vx,0,0), 0).y() : 0;
      rate += dt*(command-rate)/.08;
      pitch += dt*rate;
      vx += dt*9.80665*std::tan(pitch);
      x += dt*vx;
      gate.observe(t, t >= .65 && std::abs(x) <= .75 && std::abs(vx) <= .3 &&
          std::abs(pitch) <= 5*M_PI/180 && std::abs(rate) <= 5*M_PI/180, .1);
      settled = settled || gate.ready(.3);
    }
    EXPECT_EQ(settled, feedback);
  }
}
TEST(Plans, BalancedPairsAndPreservedCoupledSigns) {
  const std::array<double,3> rates{{.2,.3,.25}};
  auto first = identification::pulsePlan(rates,.015,3);
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
TEST(Plans, PulsePlanSingleDuration) {
  const auto plan=identification::pulsePlan({{.2,.2,.25}},.015,2);
  ASSERT_EQ(plan.size(),8u);
  for (const auto& g : plan) {
    ASSERT_EQ(g.pulses.size(),2u);
    int active=0;
    for (int i=0;i<4;++i) {
      EXPECT_DOUBLE_EQ(g.pulses[0][i],-g.pulses[1][i]);
      if (g.pulses[0][i]!=0) ++active;
    }
    EXPECT_EQ(active,1);
    EXPECT_DOUBLE_EQ(g.hold_s,0.);  // Use the FSM pulse_hold.
    EXPECT_TRUE(g.repetition==0 || g.repetition==1);
  }
}
TEST(Plans, PulsePlanThreeRoundsWithBothSigns) {
  const std::array<double,4> amplitudes{{.2,.2,.25,.015}};
  const std::array<std::string,4> channels{{"roll","pitch","yaw","thrust"}};
  const auto plan=identification::pulsePlan({{.2,.2,.25}},.015,3);
  ASSERT_EQ(plan.size(),12u);
  for (size_t i=0; i<plan.size(); ++i) {
    const auto& g=plan[i];
    EXPECT_EQ(g.repetition,static_cast<int>(i/4));
    EXPECT_EQ(g.channel,channels[i%4]);
    EXPECT_DOUBLE_EQ(g.hold_s,0.);
    ASSERT_EQ(g.pulses.size(),2u);
    for (size_t axis=0;axis<4;++axis) {
      EXPECT_DOUBLE_EQ(g.pulses[0][axis],axis==i%4 ? amplitudes[axis] : 0.);
      EXPECT_DOUBLE_EQ(g.pulses[1][axis],-g.pulses[0][axis]);
    }
  }
}
TEST(Settling, OnlyContinuousFreshSamplesCount) {
  identification::SettlingGate gate;
  gate.observe(1.,true,.1); gate.observe(1.05,true,.1);
  EXPECT_TRUE(gate.ready(.04));
  gate.observe(1.05,true,.1); EXPECT_FALSE(gate.ready(.04)); // duplicate
  gate.observe(1.1,true,.1); gate.observe(1.5,true,.1);
  EXPECT_FALSE(gate.ready(.04)); // feedback gap
  gate.observe(1.55,false,.1); EXPECT_FALSE(gate.ready(.04)); // outside gate
  gate.observe(.5,true,.1); gate.observe(.55,true,.1);
  EXPECT_TRUE(gate.ready(.04));
  gate.observe(.4,true,.1); EXPECT_FALSE(gate.ready(.04)); // clock reset
  EXPECT_FALSE(identification::freshSample(.2,.02,.1,.5));
  EXPECT_FALSE(identification::freshSample(.02,.6,.1,.5));
  EXPECT_TRUE(identification::freshSample(.02,.02,.1,.5));
}
TEST(Settling, AngularVibrationDoesNotPreventHoverButSustainedRotationDoes) {
  // 零均值 105 Hz 波动：原始合量反复超过 5 deg/s，低频机体转动为零。
  const double limit = 5*M_PI/180;
  identification::AngularRateFilter filter;
  identification::SettlingGate raw_gate, filtered_gate;
  bool raw_ready = false, filtered_ready = false;
  for (int i=0; i<400; ++i) {
    const double t = i*.005;
    const Eigen::Vector3d rate(8*M_PI/180*std::sin(2*M_PI*105*t),
                               10*M_PI/180*std::cos(2*M_PI*105*t), 0);
    const double filtered = filter.observe(t, rate, .05, .1);
    raw_gate.observe(t, rate.norm() <= limit, .1);
    filtered_gate.observe(t, filtered <= limit, .1);
    raw_ready |= raw_gate.ready(.3);
    filtered_ready |= filtered_gate.ready(.3);
  }
  EXPECT_FALSE(raw_ready);
  EXPECT_TRUE(filtered_ready);
  // 持续转动必须阻止启动，不能被当作高频波动消掉。
  filtered_gate.reset();
  for (int i=400; i<800; ++i) {
    const double t = i*.005;
    const double filtered = filter.observe(t, Eigen::Vector3d(0,0,12*M_PI/180), .05, .1);
    filtered_gate.observe(t, filtered <= limit, .1);
    EXPECT_FALSE(filtered_gate.ready(.3));
  }
}
TEST(Settling, AngularFilterRestartsOnFeedbackDiscontinuity) {
  identification::AngularRateFilter filter;
  const Eigen::Vector3d spinning(.2,0,0), stopped = Eigen::Vector3d::Zero();
  EXPECT_DOUBLE_EQ(filter.observe(1., spinning, .05, .1), .2);
  filter.observe(1.02, stopped, .05, .1);
  EXPECT_DOUBLE_EQ(filter.observe(1.2, spinning, .05, .1), .2); // gap
  filter.observe(1.22, stopped, .05, .1);
  EXPECT_DOUBLE_EQ(filter.observe(1.22, spinning, .05, .1), .2); // duplicate
  filter.observe(1.24, stopped, .05, .1);
  EXPECT_DOUBLE_EQ(filter.observe(.5, spinning, .05, .1), .2); // clock reset
  filter.observe(.52, stopped, .05, .1);
  filter.reset();
  EXPECT_DOUBLE_EQ(filter.observe(.54, spinning, .05, .1), .2); // rejected sample
}
int main(int argc,char**argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
