#include <cmath>

#include "costmap_node.hpp"

const double sideLength = 30.0; // in meters
const double resolution = 0.1;   // in meters
const int occupied = 100;
const int partially_occupied = 50;

static inline bool in_bounds(int i, int j, int h, int w) {
  return (i >= 0 && i < h && j >= 0 && j < w);
}

// traces cells beyond a hit along the same ray and tags them as -1
// note: we assume the robot is at the center of the grid (sideLength/2, sideLength/2),
//       which matches how x,y are computed below.
static void trace_behind_ray(std::vector<std::vector<int>>& costmap,
                             int hit_i, int hit_j,
                             int robot_i, int robot_j) {
  const int H = static_cast<int>(costmap.size());
  const int W = static_cast<int>(costmap[0].size());

  // direction from robot to hit in index space
  double vx = static_cast<double>(hit_j - robot_j);
  double vy = static_cast<double>(hit_i - robot_i);
  double len = std::hypot(vx, vy);
  if (len < 1e-6) return;

  // normalize step to roughly one cell per advance
  vx /= len;
  vy /= len;

  // start just beyond the hit cell
  double x = static_cast<double>(hit_j) + vx;
  double y = static_cast<double>(hit_i) + vy;

  while (true) {
    int jj = static_cast<int>(std::round(x));
    int ii = static_cast<int>(std::round(y));
    if (!in_bounds(ii, jj, H, W)) break;

    // don't override occupied cells (e.g., inflated or other hits)
    if (costmap[ii][jj] != occupied) {
      costmap[ii][jj] = -1;
    }

    x += vx;
    y += vy;
  }
}

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // init publisher
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // init subscriber
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}
 
// publish the occupancy grid
void CostmapNode::publishOccupancyGrid(std::vector<std::vector<int>> costmap) {
  auto message = nav_msgs::msg::OccupancyGrid();
  message.header.stamp = this->now();
  message.header.frame_id = "map";
  message.info.resolution = resolution; // meters
  message.info.origin.position.x = -(sideLength/2); // meters
  message.info.origin.position.y = -(sideLength/2); // meters
  message.info.width  = costmap[0].size();
  message.info.height = costmap.size();
  message.data.resize(message.info.width * message.info.height);

  for (size_t i = 0; i < costmap.size(); i++) {
    for (size_t j = 0; j < costmap[i].size(); j++) {
      // occupancygrid expects int8; -1 is fine here
      message.data[i * message.info.width + j] = static_cast<int8_t>(costmap[i][j]);
    }
  }

  // RCLCPP_INFO(this->get_logger(), "publishing occupancy grid");
  occupancy_grid_pub_->publish(message);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  const int H = static_cast<int>(sideLength / resolution);
  const int W = static_cast<int>(sideLength / resolution);
  std::vector<std::vector<int>> costmap (H, std::vector<int>(W, 0));

  // robot assumed at the center of the grid in map coords based on how x,y are computed
  const double robot_x_m = sideLength / 2.0;
  const double robot_y_m = sideLength / 2.0;
  const int robot_j = static_cast<int>(std::floor(robot_x_m / resolution));
  const int robot_i = static_cast<int>(std::floor(robot_y_m / resolution));

  for (size_t i = 0; i < scan->ranges.size(); i++) {
    const double r = scan->ranges[i];
    if (std::isinf(r) || std::isnan(r)) continue;

    const double ang = scan->angle_min + i * scan->angle_increment;
    const double x = r * std::cos(ang) + robot_x_m;
    const double y = r * std::sin(ang) + robot_y_m;

    if (x < 0 || x >= sideLength || y < 0 || y >= sideLength) {
      continue; // skip out-of-bounds
    }

    const int j = static_cast<int>(std::floor(x / resolution));
    const int ii = static_cast<int>(std::floor(y / resolution));
    if (!in_bounds(ii, j, H, W)) continue;

    // mark the hit as occupied
    costmap[ii][j] = occupied;

    // mark all cells beyond the hit along the same ray as -1
    // this prevents the memory map from being overwritten beyond visible range while turning
    trace_behind_ray(costmap, ii, j, robot_i, robot_j);
  }

  // inflate obstacles in a radius of 0.5m
  int inflation_radius = static_cast<int>(std::ceil(0.5 / resolution));
  for (int ii = 0; ii < H; ii++) {
    for (int j = 0; j < W; j++) {
      if (costmap[ii][j] == occupied) {
        for (int di = -inflation_radius; di <= inflation_radius; di++) {
          for (int dj = -inflation_radius; dj <= inflation_radius; dj++) {
            int ni = ii + di, nj = j + dj;
            if (in_bounds(ni, nj, H, W) &&
                costmap[ni][nj] != occupied &&
                (di*di + dj*dj) <= (inflation_radius * inflation_radius)) {
              // don't overwrite "behind ray" cells; leave them as -1
              if (costmap[ni][nj] != -1) {
                costmap[ni][nj] = partially_occupied;
              }
            }
          }
        }
      }
    }
  }

  // publish the occupancy grid
  publishOccupancyGrid(costmap);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
