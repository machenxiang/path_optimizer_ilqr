#include "a_star.h"

#include <algorithm>

namespace a_star {

AStarPlanner::AStarPlanner(const grid_map::GridMap& grid_map,
                           const std::string& obstacle_layer,
                           double obstacle_threshold)
    : grid_map_(grid_map),
      obstacle_layer_(obstacle_layer),
      obstacle_threshold_(obstacle_threshold),
      heuristic_weight_(1.0),
      allow_diagonal_(true) {}

std::vector<grid_map::Position> AStarPlanner::plan(
    const grid_map::Position& start, const grid_map::Position& end) {
  // 检查地图有效性
  if (!grid_map_.exists(obstacle_layer_)) {
    ROS_ERROR("Obstacle layer '%s' does not exist!", obstacle_layer_.c_str());
    return {};
  }

  // 转换坐标到索引
  grid_map::Index start_index, end_index;
  if (!grid_map_.getIndex(start, start_index)) {
    ROS_WARN("Start position is outside the map!");
    return {};
  }
  if (!grid_map_.getIndex(end, end_index)) {
    ROS_WARN("End position is outside the map!");
    return {};
  }

  // 检查起点和终点是否有效
  if (isObstacle(start_index)) {
    ROS_WARN("Start position is in an obstacle!");
    return {};
  }
  if (isObstacle(end_index)) {
    ROS_WARN("End position is in an obstacle!");
    return {};
  }

  // 使用优先队列存储待探索节点
  std::priority_queue<Node*, std::vector<Node*>, CompareNodePtr> open_set;

  // 使用unordered_map存储所有节点，key为grid_map::Index
  std::unordered_map<grid_map::Index, Node*, IndexHash, IndexEqual> all_nodes;

  // 创建起点节点
  Node* start_node = new Node{
      start_index, 0.0, calculateHeuristic(start_index, end_index), nullptr};
  open_set.push(start_node);
  all_nodes[start_index] = start_node;

  while (!open_set.empty()) {
    // 获取当前最佳节点
    Node* current = open_set.top();
    open_set.pop();

    // 检查是否到达终点
    if (current->index(0) == end_index(0) &&
        current->index(1) == end_index(1)) {
      auto path = reconstructPath(current);

      // 清理所有分配的节点
      for (auto& pair : all_nodes) {
        delete pair.second;
      }

      return path;
    }

    // 获取邻居节点
    for (const auto& neighbor_index : getNeighbors(current->index)) {
      // 跳过无效节点和障碍物
      if (!isValid(neighbor_index) || isObstacle(neighbor_index)) {
        continue;
      }

      // 计算新的g_cost
      double move_cost = (neighbor_index(0) == current->index(0) ||
                          neighbor_index(1) == current->index(1))
                             ? 1.0
                             : std::sqrt(2.0);
      double tentative_g_cost = current->g_cost + move_cost;

      // 检查是否已经访问过该节点
      auto it = all_nodes.find(neighbor_index);
      if (it == all_nodes.end()) {
        // 新节点
        Node* neighbor_node =
            new Node{neighbor_index, tentative_g_cost,
                     calculateHeuristic(neighbor_index, end_index), current};
        open_set.push(neighbor_node);
        all_nodes[neighbor_index] = neighbor_node;
      } else if (tentative_g_cost < it->second->g_cost) {
        // 找到更优路径，更新节点
        it->second->g_cost = tentative_g_cost;
        it->second->parent = current;
        // 需要重新排序优先队列
        std::priority_queue<Node*, std::vector<Node*>, CompareNodePtr> temp;
        while (!open_set.empty()) {
          temp.push(open_set.top());
          open_set.pop();
        }
        open_set = std::move(temp);
      }
    }
  }

  // 清理内存
  for (auto& pair : all_nodes) {
    delete pair.second;
  }

  ROS_WARN("No path found!");
  return {};
}

bool AStarPlanner::isObstacle(const grid_map::Index& index) const {
  if (!isValid(index)) return true;
  float value = grid_map_.at(obstacle_layer_, index);
  return value < obstacle_threshold_;
}

bool AStarPlanner::isValid(const grid_map::Index& index) const {
  return index(0) >= 0 && index(0) < grid_map_.getSize()(0) && index(1) >= 0 &&
         index(1) < grid_map_.getSize()(1);
}

double AStarPlanner::calculateHeuristic(const grid_map::Index& a,
                                        const grid_map::Index& b) const {
  double dx = a(0) - b(0);
  double dy = a(1) - b(1);
  return heuristic_weight_ * std::sqrt(dx * dx + dy * dy);
}

std::vector<grid_map::Index> AStarPlanner::getNeighbors(
    const grid_map::Index& current) const {
  std::vector<grid_map::Index> neighbors;
  neighbors.reserve(allow_diagonal_ ? 8 : 4);

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;  // 跳过当前节点

      if (!allow_diagonal_ && dx != 0 && dy != 0) continue;  // 跳过对角线

      grid_map::Index neighbor(current(0) + dx, current(1) + dy);
      neighbors.push_back(neighbor);
    }
  }

  return neighbors;
}

std::vector<grid_map::Position> AStarPlanner::reconstructPath(
    Node* end_node) const {
  std::vector<grid_map::Position> path;

  Node* current = end_node;
  while (current != nullptr) {
    grid_map::Position pos;
    grid_map_.getPosition(current->index, pos);
    path.push_back(pos);
    current = current->parent;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

void AStarPlanner::cleanup(
    std::priority_queue<Node*, std::vector<Node*>, CompareNodePtr>& open_set) {
  while (!open_set.empty()) {
    delete open_set.top();
    open_set.pop();
  }
}

}  // namespace a_star