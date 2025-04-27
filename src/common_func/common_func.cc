#include "common_func.h"
namespace common {

FindBoundary::FindBoundary(const grid_map::GridMap& grid_map,
                           const std::string& obstacle_layer,
                           double obstacle_threshold)
    : grid_map_(grid_map),
      obstacle_layer_(obstacle_layer),
      obstacle_threshold_(obstacle_threshold) {}

std::pair<std::vector<grid_map::Position>, std::vector<grid_map::Position>>
FindBoundary::FindPathBoundaries(const std::vector<grid_map::Position>& path,
                                 double search_radius) {
  std::vector<grid_map::Position> left_boundary;
  std::vector<grid_map::Position> right_boundary;
  if (path.empty()) {
    return {left_boundary, right_boundary};
  }

  // 处理第一个点
  if (path.size() > 1) {
    auto boundaries =
        FindBoundaryForPoint(path[0], path[0], path[1], search_radius);
    left_boundary.push_back(boundaries[0]);
    right_boundary.push_back(boundaries[1]);
  }

  // 处理中间点
  for (size_t i = 1; i < path.size() - 1; ++i) {
    auto boundaries =
        FindBoundaryForPoint(path[i], path[i - 1], path[i + 1], search_radius);
    left_boundary.push_back(boundaries[0]);
    right_boundary.push_back(boundaries[1]);
  }

  // 处理最后一个点
  if (path.size() > 1) {
    auto boundaries = FindBoundaryForPoint(path.back(), path[path.size() - 2],
                                           path.back(), search_radius);
    left_boundary.push_back(boundaries[0]);
    right_boundary.push_back(boundaries[1]);
  }

  return {left_boundary, right_boundary};
}

std::vector<grid_map::Position> FindBoundary::FindBoundaryForPoint(
    const grid_map::Position& point, const grid_map::Position& prev_point,
    const grid_map::Position& next_point,
    double search_radius) {  // 计算路径方向
  grid_map::Position forward_dir;
  if (next_point != point) {
    forward_dir = (next_point - point).normalized();
  } else {
    forward_dir = (point - prev_point).normalized();
  }

  // 计算左右方向
  grid_map::Position left_dir(-forward_dir.y(), forward_dir.x());
  grid_map::Position right_dir(forward_dir.y(), -forward_dir.x());

  // 搜索左右边界
  grid_map::Position left_bound = point;
  grid_map::Position right_bound = point;

  // 搜索左侧边界
  for (double d = 0.1; d <= search_radius; d += 0.1) {
    grid_map::Position test_pos = point + left_dir * d;
    grid_map::Index test_idx;
    if (grid_map_.getIndex(test_pos, test_idx) && !IsObstacle(test_idx)) {
      left_bound = test_pos;
    } else {
      break;
    }
  }

  // 搜索右侧边界
  for (double d = 0.1; d <= search_radius; d += 0.1) {
    grid_map::Position test_pos = point + right_dir * d;
    grid_map::Index test_idx;
    if (grid_map_.getIndex(test_pos, test_idx) && !IsObstacle(test_idx)) {
      right_bound = test_pos;
    } else {
      break;
    }
  }

  return {left_bound, right_bound};
}
bool FindBoundary::IsObstacle(const grid_map::Index& index)  {
  if (!IsValid(index)) return true;
  float value = grid_map_.at(obstacle_layer_, index);
  return value < obstacle_threshold_;
}

bool FindBoundary::IsValid(const grid_map::Index& index)  {
  return index(0) >= 0 && index(0) < grid_map_.getSize()(0) && index(1) >= 0 &&
         index(1) < grid_map_.getSize()(1);
}

}  // namespace common