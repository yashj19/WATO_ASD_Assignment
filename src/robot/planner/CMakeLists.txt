cmake_minimum_required(VERSION 3.10)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Create the planner library
add_library(planner_lib
  src/planner_core.cpp)

target_include_directories(planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs)

# Create the planner node executable
add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node planner_lib)

# Install targets
install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()