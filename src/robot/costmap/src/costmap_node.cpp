#include <chrono>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize publishers and subscribers
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialized ROS Constructs");

  // Initialize costmap parameters
  double resolution = 0.1;
  int width = 100;
  int height = 100;
  geometry_msgs::msg::Pose origin;
  origin.position.x = -5.0;
  origin.position.y = -5.0;
  origin.orientation.w = 1.0;
  double inflation_radius = 1.0;

  costmap_.initCostmap(resolution, width, height, origin, inflation_radius);

  RCLCPP_INFO(this->get_logger(), "Initialized Costmap Core");
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  // Update the costmap according to the laser scan
  costmap_.updateCostmap(scan);
  // Publish the costmap
  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = scan->header;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}