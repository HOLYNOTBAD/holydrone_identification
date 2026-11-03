#pragma once
#include <array>
#include <string>
#include <vector>

namespace identification {
// 波形采用 FLU [wx, wy, wz, delta_thrust]，单位 rad/s 和归一化推力增量。
// 一组内连续执行所有脉冲；统一 FSM 在组间回中，不让波形层发布控制消息。
using Pulse = std::array<double, 4>;
struct Group {
  std::string channel;
  int repetition;
  std::vector<Pulse> pulses;
  double hold_s = 0;  // 0 uses the legacy global pulse_hold
  double sine_hz = 0;  // positive: one sine segment with amplitude in pulses[0]
};
std::vector<Group> firstOrderPlan(const std::array<double, 3>& rates, double thrust, int repetitions);
std::vector<Group> secondOrderPlan(const std::array<double, 3>& rates, double thrust, int repetitions,
                                   const std::vector<double>& durations);
std::vector<Group> physicsPlan(const std::array<double, 3>& rates, double thrust, int repetitions,
                               const std::vector<double>& durations = {});
std::vector<Group> withYawSweep(const std::vector<Group>& plan, const std::vector<double>& frequencies,
                                int cycles, double amplitude);
double waveformScale(const Group& group, double elapsed);
}
