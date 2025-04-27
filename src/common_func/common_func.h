#pragma once
#include <ros/ros.h>

#include <cmath>
#include <grid_map_core/grid_map_core.hpp>
#include <queue>
#include <unordered_map>
#include <vector>

namespace common {

class FindBoundary {
 public:
  FindBoundary(const grid_map::GridMap& grid_map,
               const std::string& obstacle_layer = "obstacle",
               double obstacle_threshold = 0.5);
  ~FindBoundary() {};
  std::pair<std::vector<grid_map::Position>, std::vector<grid_map::Position>>
  FindPathBoundaries(const std::vector<grid_map::Position>& path,
                     double search_radius = 1.0);

 private:
  std::vector<grid_map::Position> FindBoundaryForPoint(
      const grid_map::Position& point, const grid_map::Position& prev_point,
      const grid_map::Position& next_point, double search_radius);
  bool IsObstacle(const grid_map::Index& index);
  bool IsValid(const grid_map::Index& index);

 private:
  const grid_map::GridMap& grid_map_;
  std::string obstacle_layer_;
  double obstacle_threshold_;
};
}  // namespace common