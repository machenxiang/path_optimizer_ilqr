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

}  // namespace osqp_planning