#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

MapMemoryNode::MapMemoryNode() 
  : Node("map_memory"), 
    map_memory_(robot::MapMemoryCore(this->get_logger())) {
  
  // Load ROS2 yaml parameters
  processParameters();

  // Subscribe to local costmap
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_,
    10,
    std::bind(&MapMemoryNode::localCostmapCallback, this, std::placeholders::_1)
  );

  // Subscribe to odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // Publish a global costmap for downstream path planning
  global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_,
    10
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(map_pub_rate_),
    std::bind(&MapMemoryNode::timerCallback, this)
  );

  map_memory_.initMapMemory(
    resolution_, 
    width_, 
    height_, 
    origin_
  );

  RCLCPP_INFO(this->get_logger(), "Initialized Map Memory Core");
}

void MapMemoryNode::processParameters() {
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("local_costmap_topic", "/costmap");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<int>("map_pub_rate", 500);
  this->declare_parameter<double>("update_distance", 1.0);
  this->declare_parameter<double>("global_map.resolution", 0.1);
  this->declare_parameter<int>("global_map.width", 300);
  this->declare_parameter<int>("global_map.height", 300);
  this->declare_parameter<double>("global_map.origin.position.x", -15.0);
  this->declare_parameter<double>("global_map.origin.position.y", -15.0);
  this->declare_parameter<double>("global_map.origin.orientation.w", 1.0);

  // Retrieve parameters and store them in member variables
  local_costmap_topic_ = this->get_parameter("local_costmap_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  map_pub_rate_ = this->get_parameter("map_pub_rate").as_int();
  update_distance_ = this->get_parameter("update_distance").as_double();
  resolution_ = this->get_parameter("global_map.resolution").as_double();
  width_ = this->get_parameter("global_map.width").as_int();
  height_ = this->get_parameter("global_map.height").as_int();
  origin_.position.x = this->get_parameter("global_map.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("global_map.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("global_map.origin.orientation.w").as_double();
}

void MapMemoryNode::localCostmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {

  bool all_zero = std::all_of(msg->data.begin(), msg->data.end(),
                            [](int8_t val) { return val == 0; });
  if (all_zero) {
    RCLCPP_INFO(this->get_logger(), "All elements in the array are zero.");
    return;
  }

  // Check how far the robot has moved since last update
  if (!std::isnan(last_robot_x_))
  {
    double dist = std::hypot(robot_x_ - last_robot_x_, robot_y_ - last_robot_y_);
    if (dist < update_distance_)
    {
      // Robot hasn’t moved enough, skip updating the global map
      return;
    }
  }

  // Update last position
  last_robot_x_ = robot_x_;
  last_robot_y_ = robot_y_;

  // Update the global map
  map_memory_.updateMap(msg, robot_x_, robot_y_, robot_theta_);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  tf2::Quaternion q(qx, qy, qz, qw);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_theta_ = yaw;
}

void MapMemoryNode::timerCallback() {
  // Publish the map every map_pub_rate [ms]
  nav_msgs::msg::OccupancyGrid map_msg = *map_memory_.getMapData();
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = "sim_world";
  global_costmap_pub_->publish(map_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}