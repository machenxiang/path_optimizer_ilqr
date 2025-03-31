#include "path_planning_interface.h"

namespace osqp_planning {
constexpr double kDeltaS = 0.5;

bool PathPlanningInterface::Run(std::vector<PathPlanning::XYPosition>* res) {
  if (free_space_ptr_ == nullptr || ref_line_ptr_ == nullptr ||
      res == nullptr) {
    return false;
  }
  SLPoints sl_points, low_boundary, up_boundary;
  std::vector<std::pair<double, double>> path_boundary;
  const double length = ref_line_ptr_->length();

  for (double s = 0.0; s < length; s = s + kDeltaS) {
    sl_points.emplace_back(SLPoint(s, 0.0));
  }
  for (const auto& pt : free_space_ptr_->boundary_points()) {
    PathPlanning::PathPoint p1, p2;
    p1.x = pt.lb_xy.x;
    p1.y = pt.lb_xy.y;

    auto sl_p1 = ref_line_ptr_->get_projection(p1);
    p2.x = pt.ub_xy.x;
    p2.y = pt.ub_xy.y;
    auto sl_p2 = ref_line_ptr_->get_projection(p2);
    low_boundary.emplace_back(SLPoint(sl_p1.s, sl_p1.l));
    up_boundary.emplace_back(SLPoint(sl_p2.s, sl_p2.l));
  }
  path_boundary.reserve(sl_points.size());
  size_t l_min_index = 0LU, u_min_index = 0LU;
  // get path boundary
  for (size_t i = 0; i < sl_points.size(); i++) {
    const auto& p = sl_points.at(i);
    double l_min_dis = std::numeric_limits<double>::max();
    double u_min_dis = std::numeric_limits<double>::max();

    for (size_t j = l_min_index; j < low_boundary.size(); j++) {
      const auto& boundary = low_boundary.at(i);
      double dis = std::abs(p.s - boundary.s);
      if (dis < l_min_dis) {
        l_min_dis = dis;
        l_min_index = j;
      }
    }
    for (size_t j = u_min_index; j < up_boundary.size(); j++) {
      const auto& boundary = up_boundary.at(i);
      double dis = std::abs(p.s - boundary.s);
      if (dis < l_min_dis) {
        u_min_dis = dis;
        u_min_index = j;
      }
    }
    path_boundary.emplace_back(std::make_pair(low_boundary.at(l_min_index).l,
                                              up_boundary.at(u_min_index).l));
  }
  // formulate osqp proble
  const auto& kNumKnots = sl_points.size();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, kDeltaS,
                                                  end_state);
  // 设置终点状态
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
  // 为路径参考(path
  // reference)设置高斯权重分布。主要优点
  // 1.对路径参考部分进行非均匀加权，中间部分影响最大
  // 2.保持非路径参考部分的权重为0
  // 3.平滑过渡的权重分布，避免突变
  {
    const double peak_value = config_.path_reference_l_weight;
    const double peak_value_x = 0.5 * static_cast<double>(kNumKnots) * kDeltaS;
    std::vector<double> weight_x_ref_vec;
    for (size_t i = 0; i < kNumKnots; ++i) {
      // Gaussian weighting
      const double x = static_cast<double>(i) * kDeltaS;
      weight_x_ref_vec.at(i) = GaussianWeighting(x, peak_value, peak_value_x);
      std::cout << "i: " << i << ", weight: " << weight_x_ref_vec.at(i)
                << std::endl;
    }
    std::vector<double> path_reference_l_ref(kNumKnots, 0.0);
    piecewise_jerk_problem.set_x_ref(std::move(weight_x_ref_vec),
                                     path_reference_l_ref);
  }
  // set weight
  {
    piecewise_jerk_problem.set_weight_x(config_.l_weight);
    piecewise_jerk_problem.set_weight_dx(config_.dl_weight);
    piecewise_jerk_problem.set_weight_ddx(config_.ddl_weight);
    piecewise_jerk_problem.set_weight_dddx(config_.dddl_weight);

    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});
  }
  // set bound
  {
    piecewise_jerk_problem.set_x_bounds(path_boundary);
    piecewise_jerk_problem.set_dx_bounds(
        -config_.lateral_derivative_bound_default,
        config_.lateral_derivative_bound_default);
    std::vector<std::pair<double, double>> ddl_bounds;
    ddl_bounds.reserve(kNumKnots);
    // 源自Optimal Vehicle Path Planning Using Quadratic Optimization for
    // Baidu Apollo Open Platform
    const double lat_acc_bound =
        std::tan(veh_prama_.max_steer_angle / veh_prama_.steer_ratio) /
        veh_prama_.wheel_base;
    for (size_t i = 0; i < kNumKnots; ++i) {
      double s = static_cast<double>(i) * kDeltaS;
      double kappa = ref_line_ptr_->get_reference_point(s).kappa;
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }
    piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);
    // jerk bound
    const double axis_distance = veh_prama_.wheel_base;
    const double max_yaw_rate =
        veh_prama_.max_steer_angle_rate / veh_prama_.steer_ratio / 2.0;
    // TODO第一个参数需要研究一下
    const double jerk_bound =
        EstimateJerkBoundary(1, axis_distance, max_yaw_rate);
    piecewise_jerk_problem.set_dddx_bound(jerk_bound);
  }
  auto start_time = std::chrono::system_clock::now();

  bool success = piecewise_jerk_problem.Optimize(config_.max_iter);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  std::cout << "Path Optimizer used time: " << diff.count() * 1000 << " ms."
            << std::endl;

  if (!success) {
    std::cout << "fail to solve" << std::endl;
    ;
    return false;
  }
  std::vector<double> x, dx, ddx;
  x = piecewise_jerk_problem.opt_x();
  dx = piecewise_jerk_problem.opt_dx();
  ddx = piecewise_jerk_problem.opt_ddx();
  // sl---->xy
  for (size_t i = 0; i < x.size(); i++) {
    const double s = i * kDeltaS;
    const auto pt =
        ref_line_ptr_->get_xy_by_sl(PathPlanning::SLPosition(s, x.at(i)));
    res->emplace_back(pt);
  }
  return true;
}

double PathPlanningInterface::GaussianWeighting(const double x,
                                                const double peak_weighting,
                                                const double peak_weighting_x) {
  double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
  double u = peak_weighting_x * std;
  double x_updated = x * std;
  // ADEBUG << peak_weighting *
  //               exp(-0.5 * (x - peak_weighting_x) * (x - peak_weighting_x));
  // ADEBUG << Gaussian(u, std, x_updated);
  return Gaussian(u, std, x_updated);
}

double PathPlanningInterface::Gaussian(const double u, const double std,
                                       const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

double PathPlanningInterface::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate)  {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

}  // namespace osqp_planning