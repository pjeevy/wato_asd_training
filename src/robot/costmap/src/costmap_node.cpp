#include <chrono>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_node.hpp"

CostmapNode::CostmapNode() 
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) 
{
  processParameters();

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_topic_, 10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 10);

  costmap_.initCostmap(
    resolution_,
    width_,
    height_,
    origin_,
    inflation_radius_
  );

  RCLCPP_INFO(this->get_logger(), "Initialized Costmap Core");
}

void CostmapNode::processParameters() {
  this->declare_parameter<std::string>("laserscan_topic", "/lidar");
  this->declare_parameter<std::string>("costmap_topic", "/costmap");
  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 300);
  this->declare_parameter<int>("height", 300);
  this->declare_parameter<double>("origin_x", -15.0);
  this->declare_parameter<double>("origin_y", -15.0);
  this->declare_parameter<double>("origin_w", 1.0);
  this->declare_parameter<double>("inflation_radius", 1.5);

  laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  resolution_ = this->get_parameter("resolution").as_double();
  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();
  origin_.position.x = this->get_parameter("origin_x").as_double();
  origin_.position.y = this->get_parameter("origin_y").as_double();
  origin_.orientation.w = this->get_parameter("origin_w").as_double();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  costmap_.updateCostmap(scan);
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