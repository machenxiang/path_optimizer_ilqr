#include "path_planning_interface.h"

namespace osqp_planning {

bool PathPlanningInterface::Run() {
  if (free_space_ptr_ == nullptr || ref_line_ptr_ == nullptr) {
    return false;
  }
  SLPoints sl_points;
  std::vector<std::pair<double, double>> path_boundary;
  const double length = ref_line_ptr_->length();

  for (double s = 0.0; s < length; s = s + 0.5) {
    sl_points.emplace_back(SLPoint(s, 0.0));
  }
  path_boundary.reserve(sl_points.size());
  for (const auto& pt : free_space_ptr_->boundary_points()) {
    PathPlanning::PathPoint p1, p2;
    p1.x = pt.lb_xy.x;
    p1.y = pt.lb_xy.y;

    auto sl_p1 = ref_line_ptr_->get_projection(p1);
    p2.x = pt.ub_xy.x;
    p2.y = pt.ub_xy.y;
    auto sl_p2 = ref_line_ptr_->get_projection(p2);
    std::cout << "low:" << sl_p1.DebugString() << "up:" << sl_p2.DebugString();
  }

  // PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s,
  //                                                 init_state.second);
  return true;
}
}  // namespace osqp_planning