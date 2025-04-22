#include "a_star.h"
#include <algorithm>

namespace a_star {

AStarPlanner::AStarPlanner(const grid_map::GridMap& grid_map, const std::string& obstacle_layer)
    : grid_map_(grid_map), 
      obstacle_layer_(obstacle_layer),
      heuristic_weight_(1.0),
      allow_diagonal_(true) {}

std::vector<grid_map::Position> AStarPlanner::plan(const grid_map::Position& start, 
                                                 const grid_map::Position& end) {
    // 将位置转换为索引
    grid_map::Index start_index, end_index;
    if (!grid_map_.getIndex(start, start_index)) {
        ROS_WARN("Start position is outside the map!");
        return {};
    }
    if (!grid_map_.getIndex(end, end_index)) {
        ROS_WARN("End position is outside the map!");
        return {};
    }
    
    // 检查起点或终点是否在障碍物中
    if (isObstacle(start_index)) {
        ROS_WARN("Start position is in an obstacle!");
        return {};
    }
    if (isObstacle(end_index)) {
        ROS_WARN("End position is in an obstacle!");
        return {};
    }
    
    // 用于open set的优先队列
    auto cmp = [](const Node* left, const Node* right) { return *left > *right; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_set(cmp);
    
    // 用于closed set的哈希集合
    std::unordered_set<Node, NodeHash> closed_set;
    
    // 创建起点节点
    Node start_node;
    start_node.index = start_index;
    start_node.g_cost = 0.0;
    start_node.h_cost = calculateHeuristic(start_index, end_index);
    start_node.parent = nullptr;
    
    open_set.push(new Node(start_node));
    
    while (!open_set.empty()) {
        // 获取f_cost最低的节点
        Node* current = open_set.top();
        open_set.pop();
        
        // 检查是否到达目标
        if (current->index(0) == end_index(0) && current->index(1) == end_index(1)) {
            auto path = reconstructPath(current);
            
            // 清理内存
            while (!open_set.empty()) {
                delete open_set.top();
                open_set.pop();
            }
            
            return path;
        }
        
        // 添加到closed set
        closed_set.insert(*current);
        
        // 探索邻居节点
        for (const auto& neighbor_index : getNeighbors(current->index)) {
            Node neighbor;
            neighbor.index = neighbor_index;
            
            // 如果在closed set中或是障碍物则跳过
            if (closed_set.find(neighbor) != closed_set.end() || isObstacle(neighbor_index)) {
                continue;
            }
            
            // 计算暂定的g_cost
            double tentative_g_cost = current->g_cost + 
                ((neighbor_index(0) == current->index(0) || neighbor_index(1) == current->index(1)) ? 1.0 : std::sqrt(2.0)); // 对角线代价
            
            // 检查邻居是否在open set中
            bool neighbor_in_open = false;
            Node* neighbor_ptr = nullptr;
            
            // 临时存储节点
            std::vector<Node*> temp_nodes;
            while (!open_set.empty()) {
                Node* node = open_set.top();
                open_set.pop();
                temp_nodes.push_back(node);
                
                if (node->index(0) == neighbor_index(0) && node->index(1) == neighbor_index(1)) {
                    neighbor_in_open = true;
                    neighbor_ptr = node;
                    break;
                }
            }
            
            // 恢复优先队列
            for (auto node : temp_nodes) {
                open_set.push(node);
            }
            
            if (!neighbor_in_open || tentative_g_cost < neighbor_ptr->g_cost) {
                if (neighbor_in_open) {
                    neighbor_ptr->g_cost = tentative_g_cost;
                    neighbor_ptr->parent = current;
                } else {
                    Node* new_node = new Node;
                    new_node->index = neighbor_index;
                    new_node->g_cost = tentative_g_cost;
                    new_node->h_cost = calculateHeuristic(neighbor_index, end_index);
                    new_node->parent = current;
                    open_set.push(new_node);
                }
            }
        }
        
        delete current;
    }
    
    ROS_WARN("No path found!");
    return {};
}

bool AStarPlanner::isValid(const grid_map::Index& index) const {
    return index(0) >= 0 && index(0) < grid_map_.getSize()(0) &&
           index(1) >= 0 && index(1) < grid_map_.getSize()(1);
}

bool AStarPlanner::isObstacle(const grid_map::Index& index) const {
    if (!isValid(index)) return true;
    return grid_map_.at(obstacle_layer_, index) < 0.5; // 假设障碍物值 < 0.5
}

double AStarPlanner::calculateHeuristic(const grid_map::Index& current, const grid_map::Index& end) const {
    // 欧几里得距离启发式
    double dx = current(0) - end(0);
    double dy = current(1) - end(1);
    return heuristic_weight_ * std::sqrt(dx*dx + dy*dy);
}

std::vector<grid_map::Index> AStarPlanner::getNeighbors(const grid_map::Index& current) const {
    std::vector<grid_map::Index> neighbors;
    
    // 4-connected或8-connected邻居
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue; // 跳过当前节点
            
            // 如果不允许对角线移动，则跳过对角线邻居
            if (!allow_diagonal_ && dx != 0 && dy != 0) continue;
            
            grid_map::Index neighbor(current(0) + dx, current(1) + dy);
            if (isValid(neighbor)) {
                neighbors.push_back(neighbor);
            }
        }
    }
    
    return neighbors;
}

std::vector<grid_map::Position> AStarPlanner::reconstructPath(Node* end_node) const {
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

} // namespace PathPlanning