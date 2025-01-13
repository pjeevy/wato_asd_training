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
  // Initialize the constructs and their parameters
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
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      costmap_core_.convertToGrid(range, angle, x_grid, y_grid);
      costmap_core_.markObstacle(x_grid, y_grid);
    }
  }
  costmap_core_.inflateObstacles();
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  // publish the costmap as an occupancygrid message
  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header.frame_id = "map";
  msg.header.stamp = this->now();
  msg.info.resolution = 0.1;
  msg.info.width = 100;
  msg.info.height = 100;
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(100 * 100);
  const auto& costmap = costmap_core_.getCostmap();
  for (int x = 0; x < 100; ++x) {
    for (int y = 0; y < 100; ++y) {
      msg.data[y * 100 + x] = costmap[x][y];
    }
  }
  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}