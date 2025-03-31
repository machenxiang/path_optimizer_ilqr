#pragma once
#include <iostream>

#include "../path/free_space.h"
#include "../path/reference_line.h"
#include "osqp_solver_common_struct.h"
#include "piecewise_jerk_path_problem.h"
#include "../path/data_structure.h"

namespace osqp_planning {
class PathPlanningInterface {
 public:
  PathPlanningInterface(
      std::shared_ptr<PathPlanning::ReferenceLine> ref_line_ptr,
      std::shared_ptr<PathPlanning::FreeSpace> free_space_ptr)
      : ref_line_ptr_(ref_line_ptr), free_space_ptr_(free_space_ptr) {};
  ~PathPlanningInterface() {};
  bool Run(std::vector<PathPlanning::XYPosition> *res);

 private:
  double GaussianWeighting(const double x, const double peak_weighting,
                           const double peak_weighting_x);
  double Gaussian(const double u, const double std, const double x);
  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_yaw_rate);

 private:
  std::shared_ptr<PathPlanning::ReferenceLine> ref_line_ptr_ = nullptr;
  std::shared_ptr<PathPlanning::FreeSpace> free_space_ptr_ = nullptr;
  PathPlanningOptimizeConfig config_;
  VehiclePrama veh_prama_;
};
}  // namespace osqp_planning
