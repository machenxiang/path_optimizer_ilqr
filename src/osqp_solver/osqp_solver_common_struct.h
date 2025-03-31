#pragma once
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
namespace osqp_planning {

struct SLPoint {
  double s = 0.0;
  double l = 0.0;
  SLPoint() = default;
  SLPoint(double _s, double _l) : s(_s), l(_l) {}
  std::string DebugString() {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(20);
    ss << "s:" << s << ",l:" << l << "\n";
    return ss.str();
  }
};
typedef std::vector<SLPoint> SLPoints;

struct PathPlanningOptimizeConfig {
  int max_iter = 4000;
  double path_reference_l_weight = 100.0;
  double lateral_derivative_bound_default = 2.0;
  double l_weight = 1.0;
  double dl_weight = 20.0;
  double ddl_weight = 1000.0;
  double dddl_weight = 50000.0;
};
struct VehiclePrama {
  double length = 4.933;
  double width = 2.11;
  double height = 1.48;
  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.0;
  double max_deceleration = -6.0;
  double max_steer_angle = 8.20304748437;
  double max_steer_angle_rate = 8.55211;
  double steer_ratio = 16;
  double wheel_base = 2.8448;
  double wheel_rolling_radius = 0.335;
  double max_abs_speed_when_stopped = 0.2;
};

}  // namespace osqp_planning