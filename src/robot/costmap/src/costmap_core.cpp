#include "costmap_core.hpp"
#include <cmath>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void CostmapCore::initializeCostmap() {
  int size = 100;
  costmap_.resize(size);
  for (int i = 0; i < size; i++) {
    costmap_[i].resize(size);
    for (int j = 0; j < size; j++) {
      costmap_[i][j] = 0;
    }
  }
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double resolution = 0.1;
  x_grid = static_cast<int>(range * cos(angle) / resolution);
  y_grid = static_cast<int>(range * sin(angle) / resolution);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < 100 && y_grid >= 0 && y_grid < 100) {
    costmap_[x_grid][y_grid] = 100;
  }
}

void CostmapCore::inflateObstacles() {
  int inflation_radius = 10; // 1 meter / 0.1 meter per cell
  int max_cost = 100;
  std::vector<std::vector<int>> inflated_costmap = costmap_;
  for (int x = 0; x < 100; ++x) {
    for (int y = 0; y < 100; ++y) {
      if (costmap_[x][y] == 100) {
        for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
          for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < 100 && ny >= 0 && ny < 100) {
              double distance = sqrt(dx * dx + dy * dy);
              if (distance <= inflation_radius) {
                int cost = static_cast<int>(max_cost * (1 - distance / inflation_radius));
                if (cost > inflated_costmap[nx][ny]) {
                  inflated_costmap[nx][ny] = cost;
                }
              }
            }
          }
        }
      }
    }
  }
  costmap_ = inflated_costmap;
}

const std::vector<std::vector<int>>& CostmapCore::getCostmap() const {
  return costmap_;
}

}