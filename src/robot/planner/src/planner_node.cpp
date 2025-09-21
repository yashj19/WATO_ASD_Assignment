#include "planner_node.hpp"
#include <cmath>

// soft safety buffer around obstacles (meters)
static constexpr double kSafetyRadiusM = 0.6;

PlannerNode::PlannerNode() 
    : Node("planner"), 
      state_(State::WAITING_FOR_GOAL) {
    
    // subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // timer (500ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), 
        std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    // if we already have a goal, try to replan right away
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "goal reached");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            planPath();
        }
    }
}

bool PlannerNode::goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath() {
    // bail if we don't yet have both, but don't spam scary logs
    if (!goal_received_ || current_map_.data.empty()) {
        if (!goal_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "waiting for goal...");
        }
        if (current_map_.data.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "waiting for map...");
        }
        return;
    }

    // robot/goal to grid
    CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
    CellIndex end   = worldToGrid(goal_.point.x,   goal_.point.y);

    // clamp end inside map bounds just in case
    end.x = std::max(0, std::min(end.x, static_cast<int>(current_map_.info.width)  - 1));
    end.y = std::max(0, std::min(end.y, static_cast<int>(current_map_.info.height) - 1));

    // small nudge if goal cell is blocked (keep it local so we don't wander)
    if (!isValidCell(end)) {
        const int max_r = 6; // ~ a few cells
        bool fixed = false;
        for (int r = 1; r <= max_r && !fixed; ++r) {
            for (int dx = -r; dx <= r && !fixed; ++dx) {
                for (int dy = -r; dy <= r && !fixed; ++dy) {
                    CellIndex c(end.x + dx, end.y + dy);
                    if (isValidCell(c)) { end = c; fixed = true; }
                }
            }
        }
        if (!fixed) RCLCPP_WARN(this->get_logger(), "goal is in a blocked area");
    }

    // a* (kept simple like your original)
    std::vector<CellIndex> path_indices = findPathAStar(start, end);

    // to ros path
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    for (const auto& index : path_indices) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        geometry_msgs::msg::Point point = gridToWorld(index);
        pose.pose.position = point;
        path.poses.push_back(pose);
    }

    path_pub_->publish(path);
}

std::vector<CellIndex> PlannerNode::findPathAStar(const CellIndex& start, const CellIndex& goal) {
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    
    open_set.push(AStarNode(start, heuristic(start, goal)));
    g_score[start] = 0;

    while (!open_set.empty()) {
        CellIndex current = open_set.top().index;
        open_set.pop();

        if (current == goal) {
            // rebuild
            std::vector<CellIndex> path;
            while (current != start) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& neighbor : getNeighbors(current)) {
            double tentative_g = g_score[current] + 1.0; // 8-connected still ok with unit cost
            
            if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal);
                open_set.push(AStarNode(neighbor, f));
            }
        }
    }

    // no path found
    return {};
}

double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool PlannerNode::isValidCell(const CellIndex& cell) {
    // bounds
    if (cell.x < 0 || cell.y < 0 || 
        cell.x >= static_cast<int>(current_map_.info.width) || 
        cell.y >= static_cast<int>(current_map_.info.height)) {
        return false;
    }

    // base occupancy check
    const int W = static_cast<int>(current_map_.info.width);
    const int idx = cell.y * W + cell.x;
    const int8_t v = current_map_.data[idx];
    if (v >= 50) return false; // treat 50+ as blocked

    // clearance ring around cell (meters → cells)
    const int clearance_cells = static_cast<int>(std::ceil(kSafetyRadiusM / current_map_.info.resolution));
    const int H = static_cast<int>(current_map_.info.height);

    for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
        for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
            const int nx = cell.x + dx;
            const int ny = cell.y + dy;
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            const int nidx = ny * W + nx;
            const int8_t nv = current_map_.data[nidx];
            if (nv >= 50) return false; // too close to stuff
        }
    }
    return true;
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& cell) {
    std::vector<CellIndex> neighbors;
    std::vector<std::pair<int, int>> dirs = {{0,1}, {1,0}, {0,-1}, {-1,0},
                                             {1,1}, {1,-1}, {-1,1}, {-1,-1}};
    const int W = static_cast<int>(current_map_.info.width);
    const int H = static_cast<int>(current_map_.info.height);

    auto hard_blocked = [&](int x, int y)->bool {
        if (x < 0 || y < 0 || x >= W || y >= H) return true;
        return current_map_.data[y * W + x] >= 50;
    };

    for (const auto& dir : dirs) {
        const int nx = cell.x + dir.first;
        const int ny = cell.y + dir.second;

        // avoid diagonal corner-cutting: the two orthogonal steps must not be hard-blocked
        if (dir.first != 0 && dir.second != 0) {
            if (hard_blocked(cell.x + dir.first, cell.y) ||
                hard_blocked(cell.x,              cell.y + dir.second)) {
                continue;
            }
        }

        CellIndex n(nx, ny);
        if (isValidCell(n)) {
            neighbors.push_back(n);
        }
    }
    return neighbors;
}

CellIndex PlannerNode::worldToGrid(double x, double y) {
    int grid_x = static_cast<int>((x - current_map_.info.origin.position.x) / 
                                  current_map_.info.resolution);
    int grid_y = static_cast<int>((y - current_map_.info.origin.position.y) / 
                                  current_map_.info.resolution);
    return CellIndex(grid_x, grid_y);
}

geometry_msgs::msg::Point PlannerNode::gridToWorld(const CellIndex& cell) {
    geometry_msgs::msg::Point point;
    point.x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
    point.y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
    point.z = 0.0;
    return point;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
