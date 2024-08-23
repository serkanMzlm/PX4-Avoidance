#ifndef __LOCAL_PLANNER_NODE_TYPES_HPP__
#define __LOCAL_PLANNER_NODE_TYPES_HPP__

#include "local_planner/local_planner_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_trajectory_waypoint.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

using navPathMsg = nav_msgs::msg::Path;
using pointCloud2Msg = sensor_msgs::msg::PointCloud2;
using poseStampedMsg = geometry_msgs::msg::PoseStamped;
using pointStampedMsg = geometry_msgs::msg::PointStamped;
using markerArrayMsg = visualization_msgs::msg::MarkerArray;
using tfStamped = geometry_msgs::msg::TransformStamped;

using vehicleOdomMsg = px4_msgs::msg::VehicleOdometry;
using vehicleStatusMsg = px4_msgs::msg::VehicleStatus;
using vehicleCommandMsg = px4_msgs::msg::VehicleCommand;
using obstacleDistanceMsg = px4_msgs::msg::ObstacleDistance;
using trajectorySetpointMsg = px4_msgs::msg::TrajectorySetpoint;
using offboardControlModeMsg = px4_msgs::msg::OffboardControlMode;
using vehicleTrajectoryWaypointMsg = px4_msgs::msg::VehicleTrajectoryWaypoint;

typedef struct
{
    rclcpp::Subscription<pointCloud2Msg>::SharedPtr point_cloud;
    rclcpp::Subscription<vehicleOdomMsg>::SharedPtr vehicle_odom;
    rclcpp::Subscription<vehicleStatusMsg>::SharedPtr vehicle_status;

    rclcpp::Subscription<poseStampedMsg>::SharedPtr pose_goal;
    rclcpp::Subscription<markerArrayMsg>::SharedPtr goal_topic;
    rclcpp::Subscription<pointStampedMsg>::SharedPtr clicked_point;
} Sub_t;

typedef struct
{
    rclcpp::Publisher<obstacleDistanceMsg>::SharedPtr obs_distance;
    rclcpp::Publisher<vehicleCommandMsg>::SharedPtr vehicle_command;
    rclcpp::Publisher<navPathMsg>::SharedPtr vehicle_path;
    rclcpp::Publisher<trajectorySetpointMsg>::SharedPtr trajectory_setpoint;
    rclcpp::Publisher<offboardControlModeMsg>::SharedPtr offboard_control_mode;
    rclcpp::Publisher<vehicleTrajectoryWaypointMsg>::SharedPtr trajector_waypoint;
} Pub_t;

typedef struct
{
    rclcpp::TimerBase::SharedPtr cmd_loop;
    rclcpp::TimerBase::SharedPtr visual;
} TimeBase_t;

typedef struct {
    pcl::PCLPointCloud2 point_cloud;
	pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
}Pcl_t;

#endif