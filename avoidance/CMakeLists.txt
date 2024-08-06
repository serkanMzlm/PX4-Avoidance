cmake_minimum_required(VERSION 3.8)
project(avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(include)
file(GLOB_RECURSE LIB_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} ${LIB_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC 
                            "$<INSTALL_INTERFACE:include/>"
                            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")


ament_target_dependencies(${PROJECT_NAME}
						  rclcpp
						  tf2_ros
						  visualization_msgs
						  tf2_geometry_msgs	
						  pcl_conversions
						  PCL
						  px4_msgs
)

ament_export_libraries(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(
	DIRECTORY include/
	DESTINATION include
)

install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

ament_export_include_directories(include)                            
ament_package()
