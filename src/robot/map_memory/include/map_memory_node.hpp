#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void processParameters();
    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    robot::MapMemoryCore map_memory_;
    double last_robot_x_, last_robot_y_;
    double robot_x_, robot_y_, robot_theta_;
    double update_distance_;
    std::string local_costmap_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    int map_pub_rate_;
    double resolution_;
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
};

#endif