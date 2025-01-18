#include <algorithm>
#include <queue>
#include <cmath>
#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void CostmapCore::initCostmap(double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius) {
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  costmap_data_->info.origin = origin;
  costmap_data_->data.assign(width * height, -1);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(inflation_radius / resolution);

  RCLCPP_INFO(logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d",
              resolution, width, height);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const {
  // Reset the costmap to free space
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  double angle = laserscan->angle_min;
  for (size_t i = 0; i < laserscan->ranges.size(); ++i, angle += laserscan->angle_increment) {
    double range = laserscan->ranges[i];

    if (range >= laserscan->range_min && range <= laserscan->range_max) { //check for validity
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);

      // convert to grid coordinates
      int grid_x = (x - costmap_data_->info.origin.position.x) / costmap_data_->info.resolution;
      int grid_y = (y - costmap_data_->info.origin.position.y) / costmap_data_->info.resolution;

      if (grid_x >= 0 && grid_x < costmap_data_->info.width &&
        grid_y >= 0 && grid_y < costmap_data_->info.height) {
        // Mark the cell as occupied
        int index = grid_y * costmap_data_->info.width + grid_x;
        costmap_data_->data[index] = 100;  // 100 indicates an occupied cell

        // Inflate around the obstacle
        inflateObstacle(grid_x, grid_y);
      }
    }
  }
}

void CostmapCore::inflateObstacle(int origin_x, int origin_y) const {
  // bfs to mark cells within inflation radius
  std::queue<std::pair<int, int>> queue;
  queue.emplace(origin_x, origin_y);

  std::vector<std::vector<bool>> visited(costmap_data_->info.width, std::vector<bool>(costmap_data_->info.height, false));
  visited[origin_x][origin_y] = true;

  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    // neighbouring cells
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;  // skip the origin cell

        int nx = x + dx;
        int ny = y + dy;

        if (nx >= 0 && nx < costmap_data_->info.width &&
          ny >= 0 && ny < costmap_data_->info.height &&
          !visited[nx][ny]) {
          
          double distance = std::sqrt((nx - origin_x) * (nx - origin_x) + (ny - origin_y) * (ny - origin_y)) * costmap_data_->info.resolution;

          // if in radius, mark cell as inflated
          if (distance <= inflation_radius_) {
              int index = ny * costmap_data_->info.width + nx;
              if (costmap_data_->data[index] < (1 - (distance / inflation_radius_)) * 100) {
                costmap_data_->data[index] = (1 - (distance / inflation_radius_)) * 100;
              }
              queue.emplace(nx, ny);
          }

          visited[nx][ny] = true;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const {
  return costmap_data_;
}

}