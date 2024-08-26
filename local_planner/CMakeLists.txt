cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

include_directories(include)

file(GLOB_RECURSE SRC_FILE ${PROJECT_SOURCE_DIR}/src/local_planner/*.cpp)
file(GLOB_RECURSE GEO_LIB_FILE ${PROJECT_SOURCE_DIR}/src/geometry_tools/*.cpp)
file(GLOB_RECURSE AVO_LIB_FILE ${PROJECT_SOURCE_DIR}/src/avoidance/*.cpp)

add_executable(${PROJECT_NAME}_node ${SRC_FILE} ${GEO_LIB_FILE} ${AVO_LIB_FILE})

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  px4_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  PCL
  pcl_ros
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen)

install(
  DIRECTORY 
  launch
  DESTINATION  
  share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()