#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>

namespace robot
{

// ------------------- Supporting Structures -------------------
struct CellIndex {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const {
        return (x != other.x || y != other.y);
    }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode {
    CellIndex index;
    double f_score;  // f = g + h

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    std::vector<geometry_msgs::msg::PoseStamped> computeAStarPath(
        const nav_msgs::msg::OccupancyGrid &map,
        const geometry_msgs::msg::Pose &start_pose,
        const geometry_msgs::msg::Point &goal_point);

  private:
    rclcpp::Logger logger_;
    double heuristic(const CellIndex &a, const CellIndex &b);
    bool isValid(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx);
};

}  

#endif
