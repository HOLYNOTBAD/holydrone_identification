/** @brief 三阶响应激励：多时长单轴正负脉冲；每个脉冲后由 FSM 回中。 */
#include <holydrone_identification/excitation_plan.hpp>
#include <stdexcept>

namespace identification {
std::vector<Group> thirdOrderPlan(const std::array<double, 3>& rates, double thrust,
                                 int repetitions, const std::vector<double>& durations) {
  if (durations.empty()) throw std::runtime_error("third-order excitation needs pulse_durations_s");
  // 模型阶次由离线拟合决定；沿用本包多时长激励，三阶配置增加重复采样。
  return secondOrderPlan(rates, thrust, repetitions, durations);
}
}
