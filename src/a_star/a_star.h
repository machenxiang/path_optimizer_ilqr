#ifndef A_STAR_H
#define A_STAR_H

#include <ros/ros.h>

#include <cmath>
#include <grid_map_core/grid_map_core.hpp>
#include <queue>
#include <unordered_map>
#include <vector>

namespace a_star {

class AStarPlanner {
 public:
  struct Node {
    grid_map::Index index;
    double g_cost;  // 从起点到当前节点的实际代价
    double h_cost;  // 到终点的启发式代价
    Node* parent;

    double f_cost() const { return g_cost + h_cost; }

    // 用于优先队列比较
    bool operator>(const Node& other) const {
      return f_cost() > other.f_cost();
    }
  };

  // 用于比较Node指针的比较器
  struct CompareNodePtr {
    bool operator()(const Node* a, const Node* b) const { return *a > *b; }
  };

  // 用于unordered_map的自定义哈希
  struct IndexHash {
    size_t operator()(const grid_map::Index& index) const {
      return std::hash<int>()(index(0)) ^ (std::hash<int>()(index(1)) << 1);
    }
  };

  // 用于unordered_map的自定义比较
  struct IndexEqual {
    bool operator()(const grid_map::Index& a, const grid_map::Index& b) const {
      return a(0) == b(0) && a(1) == b(1);
    }
  };

  AStarPlanner(const grid_map::GridMap& grid_map,
               const std::string& obstacle_layer = "obstacle",
               double obstacle_threshold = 0.5);

  std::vector<grid_map::Position> plan(const grid_map::Position& start,
                                       const grid_map::Position& end);

  void setHeuristicWeight(double weight) { heuristic_weight_ = weight; }
  void setAllowDiagonal(bool allow) { allow_diagonal_ = allow; }

 private:
  bool isObstacle(const grid_map::Index& index) const;
  bool isValid(const grid_map::Index& index) const;
  double calculateHeuristic(const grid_map::Index& a,
                            const grid_map::Index& b) const;
  std::vector<grid_map::Index> getNeighbors(
      const grid_map::Index& current) const;
  std::vector<grid_map::Position> reconstructPath(Node* end_node) const;
  void cleanup(
      std::priority_queue<Node*, std::vector<Node*>, CompareNodePtr>& open_set);

  const grid_map::GridMap& grid_map_;
  std::string obstacle_layer_;
  double obstacle_threshold_;
  double heuristic_weight_;
  bool allow_diagonal_;
};

}  // namespace a_star

#endif  // A_STAR_H