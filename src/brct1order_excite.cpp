/** @brief 一阶响应的纯波形定义：每轮 roll/pitch/yaw/thrust 各一组 +/-。 */
#include <holydrone_identification/excitation_plan.hpp>
namespace identification {
std::vector<Group> firstOrderPlan(const std::array<double, 3>& rates, double thrust, int repetitions) {
  const char* names[] = {"roll", "pitch", "yaw", "thrust"};
  std::vector<Group> plan;
  for (int repeat = 0; repeat < repetitions; ++repeat) {
    for (int axis = 0; axis < 4; ++axis) {
      Pulse positive{}, negative{};
      positive[axis] = axis < 3 ? rates[axis] : thrust;
      negative[axis] = -positive[axis];
      plan.push_back({names[axis], repeat, {positive, negative}});
    }
  }
  return plan;
}
}
