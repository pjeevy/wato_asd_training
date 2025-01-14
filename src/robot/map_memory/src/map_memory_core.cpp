#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap) {
  // ensure the global map is initialized
  if (global_map_.data.empty()) {
    global_map_ = costmap;
    return;
  }

  // merge costmap into the global map
  for (size_t y = 0; y < costmap.info.height; ++y) {
    for (size_t x = 0; x < costmap.info.width; ++x) {
      size_t index = y * costmap.info.width + x;
      int8_t costmap_value = costmap.data[index];
      int8_t& global_value = global_map_.data[index];

      if (costmap_value != -1) { // if the cell is known in the costmap
        global_value = costmap_value;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

}
