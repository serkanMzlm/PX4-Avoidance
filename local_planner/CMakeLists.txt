cmake_minimum_required(VERSION 3.8)
project(local_planner)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

find_package (Eigen3 3.3 REQUIRED)

include_directories(include)
file(GLOB_RECURSE LIB_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/avoidance/*.cpp)

add_executable(local_planner_node ${LIB_FILE})
ament_target_dependencies(
  local_planner_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs

  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  px4_msgs

  PCL
)

target_link_libraries(local_planner_node Eigen3::Eigen)

ament_package()
