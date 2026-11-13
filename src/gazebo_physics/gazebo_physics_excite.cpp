/** @brief 物理辨识的纯波形定义；单轴每两脉冲回中，耦合四向组合分别追加反向脉冲，每对后回中。 */
#include <excitation/excitation_plan.hpp>
#include <cmath>
#include <set>
#include <stdexcept>
namespace identification {
std::vector<Group> physicsPlan(const std::array<double, 3>& rates, double thrust, int repetitions, const std::vector<double>& durations) {
  const char* names[] = {"thrust", "roll", "pitch", "yaw"};
  std::vector<Group> plan;
  if (!durations.empty()) {
    for (int repeat=0; repeat<repetitions; ++repeat) {
      auto pair = [&](const std::string& channel, Pulse p, double hold) {
        Pulse n{}; for (size_t i=0; i<4; ++i) n[i]=-p[i];
        plan.push_back({channel, repeat, {p,n}, hold});
      };
      // Different pulse widths produce different spectral content without increasing amplitude.
      for (double hold : durations) for (int experiment=0; experiment<4; ++experiment) {
        const int axis=experiment==0 ? 3 : experiment-1;
        Pulse p{}; p[axis]=axis<3 ? rates[axis] : thrust;
        pair(names[experiment],p,hold);
      }
      int pattern=repeat;
      for (int a=0; a<3; ++a) for (int b=a+1; b<3; ++b) for (int sign : {-1,1}) {
        Pulse p{}; p[a]=rates[a]; p[b]=sign*rates[b];
        pair("coupled",p,durations[pattern++ % durations.size()]);
      }
      const int signs[4][3]={{1,1,1},{1,-1,-1},{-1,1,-1},{-1,-1,1}};
      for (const auto& sign : signs)
        pair("coupled",Pulse{{sign[0]*rates[0],sign[1]*rates[1],sign[2]*rates[2],0}},
             durations[pattern++ % durations.size()]);
    }
    return plan;
  }
  for (int repeat = 0; repeat < repetitions; ++repeat) {
    for (int experiment = 0; experiment < 4; ++experiment) {
      const int axis = experiment == 0 ? 3 : experiment-1;
      Pulse positive{}, negative{};
      positive[axis] = axis < 3 ? rates[axis] : thrust;
      negative[axis] = -positive[axis];
      for (int pair = 0; pair < 2; ++pair)
        plan.push_back({names[experiment], repeat, {positive, negative}});
    }
    // 原四向组合覆盖三轴交叉项；每个方向增加反向平台，避免连续同向 roll 积累倾角和漂移。
    const int signs[4][3] = {{1,1,1}, {1,-1,-1}, {-1,1,-1}, {-1,-1,1}};
    for (const auto& sign : signs) {
      Pulse positive{{sign[0]*rates[0], sign[1]*rates[1], sign[2]*rates[2], 0}};
      Pulse negative{{-positive[0],-positive[1],-positive[2],0}};
      plan.push_back({"coupled", repeat, {positive, negative}});
    }
  }
  return plan;
}
std::vector<Group> withYawSweep(const std::vector<Group>& plan, const std::vector<double>& frequencies,
                                int cycles, double amplitude) {
  if (frequencies.empty() || cycles<2 || cycles>10 || !std::isfinite(amplitude) || amplitude<=0 || amplitude>.4)
    throw std::runtime_error("invalid yaw sweep amplitude/cycles/frequencies");
  for(double f:frequencies) if(!std::isfinite(f) || f<.2 || f>5)
    throw std::runtime_error("yaw sweep frequency must be within [0.2,5] Hz");
  std::vector<Group> result;
  std::set<int> replaced;
  for(const auto& group:plan) {
    if(group.channel!="yaw") { result.push_back(group); continue; }
    if(!replaced.insert(group.repetition).second) continue;
    for(double f:frequencies)
      result.push_back({"yaw",group.repetition,{Pulse{{0,0,amplitude,0}}},cycles/f,f});
  }
  return result;
}
double waveformScale(const Group& group, double elapsed) {
  if(group.sine_hz<=0) return 1.;
  if(elapsed<=0 || elapsed>=group.hold_s) return 0.;
  return std::sin(2*M_PI*group.sine_hz*elapsed);
}

}
