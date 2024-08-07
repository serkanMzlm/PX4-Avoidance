cmake_minimum_required(VERSION 3.8)
project(local_planner)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(avoidance REQUIRED)

find_package (Eigen3 3.3 REQUIRED)

include_directories(include)
file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(local_planner_node ${SRC_FILE})
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
  nav_msgs
  px4_msgs
  pcl_conversions
  PCL
  avoidance
)
target_link_libraries(local_planner_node Eigen3::Eigen yaml-cpp)

install(
  DIRECTORY 
  launch
  DESTINATION  
  share/${PROJECT_NAME}/
)

install(TARGETS
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
