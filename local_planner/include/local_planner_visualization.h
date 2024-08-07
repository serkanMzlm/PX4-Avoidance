#ifndef LOCAL_PLANNER_VISUALIZATION_H
#define LOCAL_PLANNER_VISUALIZATION_H

#include <vector>
#include <Eigen/Dense>

#include "local_planner.h"
#include "waypoint_generator.h"

#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace avoidance {

class LocalPlannerVisualization {
 public:
  /**
  * @brief      initializes all publishers used for local planner visualization
  **/
  void initializePublishers(rclcpp::Node& nh);

  /**
  * @brief       Main function which calls functions to visualize all planner
  *              output ready at the end of one planner iteration
  * @params[in]  planner, reference to the planner
  * @params[in]  newest_waypoint_position, last caluclated waypoint (smoothed)
  * @params[in]  newest_adapted_waypoint_position, last caluclated waypoint
  *              (non-smoothed)
  **/
  void visualizePlannerData(const LocalPlanner& planner, const Eigen::Vector3f& newest_waypoint_position,
                            const Eigen::Vector3f& newest_adapted_waypoint_position,
                            const Eigen::Vector3f& newest_position, const Eigen::Quaternionf& newest_orientation) const;

  /**
  * @brief       Visualization of the calculated search tree and the best path
  *              chosen
  * @params[in]  tree, the complete calculated search tree
  * @params[in]  closed_set, the closed set (all expanded nodes)
  * @params[in]  path_node_positions, the positions of all nodes belonging to
  *              the chosen best path
  **/
  void publishTree(const std::vector<TreeNode>& tree, const std::vector<int>& closed_set,
                   const std::vector<Eigen::Vector3f>& path_node_positions) const;

  /**
  * @brief       Visualization of the goal position
  * @params[in]  goal, the loaction of the goal used in the planner calculations
  **/
  void publishGoal(const geometry_msgs::msg::Point& goal) const;

  /**
  * @brief       Visualization of the 2D compression of the local pointcloud
  * @params[in]  histogram_image, data for visualization
  * @params[in]  cost_image, data for visualization
  * @params[in]  newest_waypoint_position, last calculated waypoint (smoothed)
  * @params[in]  newest_adapted_waypoint_position, last calculated waypoint
  *              (non-smoothed)
  * @params[in]  newest_pose, most recent drone pose
  **/
  void publishDataImages(const std::vector<uint8_t>& histogram_image_data, const std::vector<uint8_t>& cost_image_data,
                         const Eigen::Vector3f& newest_waypoint_position,
                         const Eigen::Vector3f& newest_adapted_waypoint_position,
                         const Eigen::Vector3f& newest_position, const Eigen::Quaternionf newest_orientation) const;

  /**
  * @brief       Visualization of the waypoint calculation
  * @params[in]  goto_position, original calculated desired direction
  * @params[in]  adapted_goto_position, desired direction adapted to the desired
  *speed
  * @params[in]  smoothed_goto_position, smoothed waypoint which is given to the
  *controller
  **/
  void visualizeWaypoints(const Eigen::Vector3f& goto_position, const Eigen::Vector3f& adapted_goto_position,
                          const Eigen::Vector3f& smoothed_goto_position) const;

  /**
  * @brief       Visualization of the actual path of the drone and the path of
  *the waypoint
  * @params[in]  last_pos, location of the drone at the last timestep
  * @params[in]  newest_pos, location of the drone at the current timestep
  * @params[in]  last_wp, location of the smoothed waypoint at the last timestep
  * @params[in]  newest_wp, location of the smoothed waypoint at the current
  *              timestep
  * @params[in]  last_adapted_wp, location of the adapted waypoint at the last
  *              timestep
  * @params[in]  newest_adapted_wp, location of the adapted waypoint at the
  *              current timestep
  **/
  void publishPaths(const Eigen::Vector3f& last_position, const Eigen::Vector3f& newest_position,
                    const Eigen::Vector3f& last_wp, const Eigen::Vector3f& newest_wp,
                    const Eigen::Vector3f& last_adapted_wp, const Eigen::Vector3f& newest_adapted_wp);

  /**
  * @brief       Visualization of the sent waypoint color coded with the mode
  *              the planner is in
  * @params[in]  wp, sent wayoint
  * @params[in]  waypoint_type, current planner mode to color code the
  *              visualization
  * @params[in]  newest_pos, location of the drone at the current timestep
  **/
  void publishCurrentSetpoint(const geometry_msgs::msg::Twist& wp, const PlannerState& waypoint_type,
                              const Eigen::Vector3f& newest_position) const;

  /**
  * @brief       Visualization of the offtrack state
  * @params[in]  closest_pt, vehicle position projection on the line previous to
  * current goal
  * @params[in]  deg60_pt, 60 degrees angle entry point to line previous to
  * current goal from current vehicle postion
  **/
  void publishOfftrackPoints(Eigen::Vector3f& closest_pt, Eigen::Vector3f& deg60_pt);

  void publishFOV(const std::vector<FOV>& fov, float max_range) const;

  void publishRangeScan(const sensor_msgs::msg::LaserScan& scan, const Eigen::Vector3f& newest_position) const;

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pointcloud_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pointcloud_size_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ground_measurement_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr original_wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr adapted_wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr smoothed_wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr complete_tree_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_actual_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_adapted_waypoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr current_waypoint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr histogram_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cost_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr deg60_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr range_scan_pub_;

  int path_length_ = 0;
};
}
#endif  // LOCAL_PLANNER_VISUALIZATION_H
