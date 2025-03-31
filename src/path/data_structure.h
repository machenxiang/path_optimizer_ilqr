#pragma once
#include <Eigen/Dense>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "solver/variable.h"
namespace PathPlanning {

using Solver::Variable;
using Solver::Node;

constexpr std::size_t N_PATH_STATE  = 3;
constexpr std::size_t N_PATH_CONTROL = 1;
constexpr std::size_t L_INDEX = 0;
constexpr std::size_t HD_INDEX = 1; // HD for heading diff.
constexpr std::size_t K_INDEX = 2;
constexpr std::size_t KR_INDEX = 0; // Control, kappa rate.

struct XYPosition {
    double x = 0.0;
    double y = 0.0;
};
using Vector = XYPosition;

struct SLPosition {
  double s = 0.0;
  double l = 0.0;
  SLPosition() : s(0.0), l(0.0) {}
  SLPosition(double _s, double _l) : s(_s), l(_l) {}
  std::string DebugString() {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(20);
    ss << "s:" << s << ",l:" << l << "\n";
    return ss.str();
  }
};

struct PathPoint : public XYPosition, SLPosition {
    double theta = 0.0;
    double kappa = 0.0;
    double dkappa = 0.0;
    double theta_diff = 0.0;
    double dl = 0.0;
    double ddl = 0.0;
};

} // namespace PathPlanning
