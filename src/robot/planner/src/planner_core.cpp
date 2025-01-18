#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) {
    // Euclidean distance
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool PlannerCore::isValid(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) {
    return idx.x >= 0 && idx.x < static_cast<int>(map.info.width) &&
           idx.y >= 0 && idx.y < static_cast<int>(map.info.height) &&
           map.data[idx.y * map.info.width + idx.x] < 50; // Threshold for occupancy
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::computeAStarPath(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::Pose &start_pose,
    const geometry_msgs::msg::Point &goal_point) {

    RCLCPP_DEBUG(logger_, "Starting A* path computation.");

    // Convert start and goal to grid indices
    int start_x = static_cast<int>((start_pose.position.x - map.info.origin.position.x) / map.info.resolution);
    int start_y = static_cast<int>((start_pose.position.y - map.info.origin.position.y) / map.info.resolution);
    CellIndex start(start_x, start_y);

    int goal_x = static_cast<int>((goal_point.x - map.info.origin.position.x) / map.info.resolution);
    int goal_y = static_cast<int>((goal_point.y - map.info.origin.position.y) / map.info.resolution);
    CellIndex goal(goal_x, goal_y);

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    open_set.emplace(start, heuristic(start, goal));
    g_score[start] = 0.0;

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        if (current.index == goal) {
            RCLCPP_INFO(logger_, "Goal reached in A*.");
            // Reconstruct path
            std::vector<geometry_msgs::msg::PoseStamped> path;
            CellIndex trace = current.index;
            while (!(trace == start)) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map"; // Set frame_id
                pose.pose.position.x = trace.x * map.info.resolution + map.info.origin.position.x;
                pose.pose.position.y = trace.y * map.info.resolution + map.info.origin.position.y;
                pose.pose.orientation.w = 1.0;
                path.push_back(pose);
                if (came_from.find(trace) == came_from.end()) {
                    RCLCPP_WARN(logger_, "Path reconstruction failed.");
                    break;
                }
                trace = came_from[trace];
            }
            std::reverse(path.begin(), path.end());
            RCLCPP_INFO(logger_, "Path reconstructed with %zu poses.", path.size());
            return path;
        }

        // 8-connected grid
        std::vector<CellIndex> neighbors = {
            CellIndex(current.index.x + 1, current.index.y),
            CellIndex(current.index.x - 1, current.index.y),
            CellIndex(current.index.x, current.index.y + 1),
            CellIndex(current.index.x, current.index.y - 1),
            CellIndex(current.index.x + 1, current.index.y + 1),
            CellIndex(current.index.x - 1, current.index.y - 1),
            CellIndex(current.index.x + 1, current.index.y - 1),
            CellIndex(current.index.x - 1, current.index.y + 1)
        };

        for (const auto &neighbor : neighbors) {
            if (!isValid(map, neighbor)) continue;

            // Use a proper step cost based on straight vs. diagonal movement
            double step_cost =
                (std::abs(neighbor.x - current.index.x) + std::abs(neighbor.y - current.index.y) == 2)
                ? std::sqrt(2.0) : 1.0;

            double tentative_g = g_score[current.index] + step_cost;
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current.index;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal);
                open_set.emplace(neighbor, f);
            }
        }
    }

    RCLCPP_WARN(logger_, "A* failed to find a path.");
    // Return empty path if no path found
    return {};
}

} // namespace robot
