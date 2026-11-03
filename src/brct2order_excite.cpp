/** @brief 二阶响应激励：单轴多时长正负双脉冲，组间由统一 FSM 回中。 */
#include <holydrone_identification/excitation_plan.hpp>
#include <cmath>
#include <stdexcept>
namespace identification {
std::vector<Group> secondOrderPlan(const std::array<double, 3>& rates, double thrust,
                                  int repetitions, const std::vector<double>& durations) {
  if (durations.empty()) throw std::runtime_error("second-order excitation needs pulse_durations_s");
  std::vector<Group> plan;
  for (int repeat = 0; repeat < repetitions; ++repeat) {
    for (double duration : durations) {
      if (!std::isfinite(duration) || duration < .1 || duration > .75)
        throw std::runtime_error("pulse_durations_s must be within [0.1, 0.75] s");
      for (auto group : firstOrderPlan(rates, thrust, 1)) {
        group.repetition = repeat;
        group.hold_s = duration;
        plan.push_back(group);
      }
    }
  }
  return plan;
}
}
