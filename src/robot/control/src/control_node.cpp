#include "control_node.hpp"
#include <cmath>
#include <algorithm> // clamp

ControlNode::ControlNode()
    : Node("control"), 
      control_(robot::ControlCore(this->get_logger())),
      lookahead_distance_(1),
      goal_tolerance_(0.1),
      linear_speed_(1.3) {
    
    // subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
    
    // publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // timer (10hz)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = msg;
    RCLCPP_INFO(this->get_logger(), "received path with %zu waypoints", msg->poses.size());
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_odom_ = msg;
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
        // stop if we don't have what we need
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    const auto& robot_pos = robot_odom_->pose.pose.position;
    
    // find first point at least lookahead_distance_ away
    for (size_t i = 0; i < current_path_->poses.size(); i++) {
        const auto& path_point = current_path_->poses[i].pose.position;
        double distance = computeDistance(robot_pos, path_point);
        if (distance >= lookahead_distance_) {
            return current_path_->poses[i];
        }
    }
    
    // if nothing far enough, use last if close enough
    if (!current_path_->poses.empty()) {
        const auto& last_point = current_path_->poses.back().pose.position;
        if (computeDistance(robot_pos, last_point) < goal_tolerance_) {
            return current_path_->poses.back();
        }
    }
    
    return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(
    const geometry_msgs::msg::PoseStamped &target) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    // robot pose
    const auto& robot_pos = robot_odom_->pose.pose.position;
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);
    
    // angle to target
    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;
    double target_angle = std::atan2(dy, dx);
    
    // steering angle (wrapped)
    double angle_diff = target_angle - robot_yaw;
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
    
    // go slower when we need a big heading change so we don't shove into obstacles
    const double turn_slowdown = std::clamp(std::cos(angle_diff), 0.0, 1.0);

    // linear and angular velocities
    cmd_vel.linear.x = linear_speed_ * turn_slowdown;
    cmd_vel.angular.z = 2.0 * angle_diff; // simple p controller

    // optional: saturate angular to something reasonable
    const double max_ang = 1.5; // rad/s
    if (cmd_vel.angular.z >  max_ang) cmd_vel.angular.z =  max_ang;
    if (cmd_vel.angular.z < -max_ang) cmd_vel.angular.z = -max_ang;
    
    return cmd_vel;
}

double ControlNode::computeDistance(
    const geometry_msgs::msg::Point &a, 
    const geometry_msgs::msg::Point &b) {
    
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    return tf2::getYaw(quat);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
