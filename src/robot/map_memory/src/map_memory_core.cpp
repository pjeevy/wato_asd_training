#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap) {
  // Ensure the global map is initialized
  if (global_map_.data.empty()) {
    global_map_ = costmap;
    return;
  }

  // Transform the costmap to the global frame (assuming identity transform for simplicity)
  // In a real implementation, you would apply the robot's current pose to transform the costmap

  // Merge the costmap into the global map
  for (size_t y = 0; y < costmap.info.height; ++y) {
    for (size_t x = 0; x < costmap.info.width; ++x) {
      size_t index = y * costmap.info.width + x;
      int8_t costmap_value = costmap.data[index];
      int8_t& global_value = global_map_.data[index];

      if (costmap_value != -1) { // If the cell is known in the costmap
        global_value = costmap_value;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

}
