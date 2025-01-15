#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <memory>

namespace robot
{

class CostmapCore {
public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initCostmap(double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius);
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

private:
    void inflateObstacle(int origin_x, int origin_y) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    double inflation_radius_;
    int inflation_cells_;
};

} // namespace robot

#endif // COSTMAP_CORE_HPP_