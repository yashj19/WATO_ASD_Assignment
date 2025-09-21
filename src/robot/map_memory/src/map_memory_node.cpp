#include "map_memory_node.hpp"
#include <chrono>
#include <memory>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

const double sideLength = 30.0; // in meters
const double resolution = 0.2;   // in meters
const int occupied = 100;
const int partially_occupied = 50;
const int totalSideUnits = static_cast<int>(sideLength / resolution);

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x(0.0), last_y(0.0), yaw(0.0), distance_threshold(0.1) {
  // init publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // init subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  // init timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MapMemoryNode::updateMap, this));

  RCLCPP_INFO(this->get_logger(), "map memory node initialized");

  // init global map
  global_map_ = nav_msgs::msg::OccupancyGrid();
  global_map_.header.frame_id = "map";
  global_map_.info.resolution = resolution; // meters
  global_map_.info.origin.position.x = -(sideLength/2); // meters
  global_map_.info.origin.position.y = -(sideLength/2); // meters
  global_map_.info.width = sideLength / resolution;
  global_map_.info.height = sideLength / resolution;

  // keep 0 as your "unknown" baseline to match current pipeline
  global_map_.data.resize(global_map_.info.width * global_map_.info.height, 0);
}

// callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
  if (initialCount < 5) {
      should_update_map_ = true; // force update for first few messages
  }
  // RCLCPP_INFO(this->get_logger(), "received new costmap");
}

// callback for odometry updates
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= distance_threshold) {
    last_x = x;
    last_y = y;
    yaw = tf2::getYaw(msg->pose.pose.orientation);
    should_update_map_ = true;
  }
}

// timer-based map update
void MapMemoryNode::updateMap() {
  RCLCPP_INFO(this->get_logger(), "timer tick with should_update_map_=%d and costmap_updated_=%d", should_update_map_, costmap_updated_);
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    RCLCPP_INFO(this->get_logger(), "publishing updated global map");
    map_pub_->publish(global_map_);
    should_update_map_ = false;
    initialCount++;
  }
}

// integrate the latest costmap into the global map
void MapMemoryNode::integrateCostmap() {
  // costmap info
  double costmap_origin_x = latest_costmap_.info.origin.position.x;
  double costmap_origin_y = latest_costmap_.info.origin.position.y;
  double costmap_res = latest_costmap_.info.resolution;
  int costmap_width = latest_costmap_.info.width;
  int costmap_height = latest_costmap_.info.height;

  // global map info
  double global_origin_x = global_map_.info.origin.position.x;
  double global_origin_y = global_map_.info.origin.position.y;
  double global_res = global_map_.info.resolution;
  int global_width = global_map_.info.width;
  int global_height = global_map_.info.height;

  // for each cell in the costmap
  for (int i = 0; i < costmap_height; ++i) {
    for (int j = 0; j < costmap_width; ++j) {
      int costmap_idx = i * costmap_width + j;
      int8_t costmap_value = latest_costmap_.data[costmap_idx];

      // skip the "behind-ray" marker so we don't overwrite the memory map outside current fov
      if (costmap_value == -1) continue;

      // costmap cell index to local (costmap) coordinates (meters)
      double cx = costmap_origin_x + j * costmap_res + costmap_res / 2.0;
      double cy = costmap_origin_y + i * costmap_res + costmap_res / 2.0;

      // transform to global frame using robot pose
      double gx = std::cos(yaw) * cx - std::sin(yaw) * cy + last_x;
      double gy = std::sin(yaw) * cx + std::cos(yaw) * cy + last_y;

      // global coordinates to global map indices
      int map_j = static_cast<int>((gx - global_origin_x) / global_res);
      int map_i = static_cast<int>((gy - global_origin_y) / global_res);

      if (map_i >= 0 && map_i < global_height && map_j >= 0 && map_j < global_width) {
        int global_idx = map_i * global_width + map_j;

        // write-through policy: take the latest costmap value if it's not -1
        // (this allows updating with better data while avoiding occluded regions)
        global_map_.data[global_idx] = costmap_value;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
