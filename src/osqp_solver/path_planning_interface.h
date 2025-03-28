#pragma once
#include <iostream>

#include "../path/free_space.h"
#include "../path/reference_line.h"
#include "osqp_solver_common_struct.h"
#include "piecewise_jerk_path_problem.h"

namespace osqp_planning {
class PathPlanningInterface {
 public:
  PathPlanningInterface(
      std::shared_ptr<PathPlanning::ReferenceLine> ref_line_ptr,
      std::shared_ptr<PathPlanning::FreeSpace> free_space_ptr)
      : ref_line_ptr_(ref_line_ptr), free_space_ptr_(free_space_ptr) {};
  ~PathPlanningInterface() {};
  bool Run();

 private:
  std::shared_ptr<PathPlanning::ReferenceLine> ref_line_ptr_ = nullptr;
  std::shared_ptr<PathPlanning::FreeSpace> free_space_ptr_ = nullptr;
};
}  // namespace osqp_planning
