#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot {

class CostmapCore {
public:
  explicit CostmapCore(const rclcpp::Logger& logger);
  void initializeCostmap();
  void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  const std::vector<std::vector<int>>& getCostmap() const;

private:
  rclcpp::Logger logger_;
  std::vector<std::vector<int>> costmap_;
  int origin_x_;
  int origin_y_;
};

} // namespace robot

#endif // COSTMAP_CORE_HPP_