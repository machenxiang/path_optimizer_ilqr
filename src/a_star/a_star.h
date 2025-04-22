#ifndef A_STAR_H
#define A_STAR_H

#include <grid_map_core/grid_map_core.hpp>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <functional>
#include <ros/ros.h>  // 添加ROS头文件

namespace a_star {

struct Node {
    grid_map::Index index;
    double g_cost;  // 从起点到当前节点的代价
    double h_cost;  // 从当前节点到终点的启发式代价
    double f_cost() const { return g_cost + h_cost; }
    Node* parent = nullptr;
    
    // 用于优先队列比较
    bool operator>(const Node& other) const {
        return f_cost() > other.f_cost();
    }
    
    // 用于unordered_set比较
    bool operator==(const Node& other) const {
        return (index(0) == other.index(0)) && (index(1) == other.index(1));  // 修改为逐个元素比较
    }
};

// Node哈希函数
struct NodeHash {
    size_t operator()(const Node& node) const {
        return std::hash<int>()(node.index(0)) ^ std::hash<int>()(node.index(1));
    }
};

class AStarPlanner {
public:
    AStarPlanner(const grid_map::GridMap& grid_map, const std::string& obstacle_layer = "obstacle");
    
    // 主规划函数
    std::vector<grid_map::Position> plan(const grid_map::Position& start, 
                                        const grid_map::Position& end);
    
    // 设置启发式权重(默认1.0)
    void setHeuristicWeight(double weight) { heuristic_weight_ = weight; }
    
    // 启用/禁用对角线移动(默认启用)
    void setAllowDiagonal(bool allow) { allow_diagonal_ = allow; }

private:
    // 辅助函数
    bool isValid(const grid_map::Index& index) const;
    bool isObstacle(const grid_map::Index& index) const;
    double calculateHeuristic(const grid_map::Index& current, const grid_map::Index& end) const;
    std::vector<grid_map::Index> getNeighbors(const grid_map::Index& current) const;
    std::vector<grid_map::Position> reconstructPath(Node* end_node) const;
    
    const grid_map::GridMap& grid_map_;
    std::string obstacle_layer_;
    double heuristic_weight_;
    bool allow_diagonal_;
};

} // namespace PathPlanning

#endif // A_STAR_H