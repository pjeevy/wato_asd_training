#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_core_(this->get_logger()) {
  // Initialize publishers and subscribers
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_core_.initializeCostmap();
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_core_.initializeCostmap();
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (std::isfinite(range)) {
      double angle = scan->angle_min + i * scan->angle_increment;
      int x_grid, y_grid;
      costmap_core_.convertToGrid(range, angle, x_grid, y_grid);
      costmap_core_.markObstacle(x_grid, y_grid);
    }
  }
  costmap_core_.inflateObstacles();
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  auto costmap = costmap_core_.getCostmap();
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = this->now();
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.info.resolution = 0.1;
  occupancy_grid.info.width = costmap.size();
  occupancy_grid.info.height = costmap[0].size();
  occupancy_grid.info.origin.position.x = -5.0; // Adjust based on your origin
  occupancy_grid.info.origin.position.y = -5.0; // Adjust based on your origin
  occupancy_grid.info.origin.position.z = 0.0;
  occupancy_grid.info.origin.orientation.w = 1.0;

  occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);
  for (size_t i = 0; i < costmap.size(); ++i) {
    for (size_t j = 0; j < costmap[i].size(); ++j) {
      occupancy_grid.data[i * costmap.size() + j] = costmap[i][j];
    }
  }

  costmap_pub_->publish(occupancy_grid);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}