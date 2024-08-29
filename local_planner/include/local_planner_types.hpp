#ifndef __LOCAL_PLANNER_NODE_TYPES_HPP__
#define __LOCAL_PLANNER_NODE_TYPES_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>

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

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float data[3];
} Vec3_t;

typedef struct
{
    union
    {
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
        float quaternion[4];
    };

    union
    {
        struct
        {
            float roll;
            float pitch;
            float yaw;
        };
        float euler[3];
    };
} Orientation_t;

typedef struct
{
    Vec3_t position;
    Vec3_t velocity;
    Orientation_t orientation;
} State_t;

enum class ArmState
{
    ARM,
    DISARM
};

struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : e(e_), z(z_), r(r_){};
  PolarPoint() : e(0.0f), z(0.0f), r(0.0f){};
  float e;
  float z;
  float r;
};

struct FOV {
  FOV() : yaw_deg(0.f), pitch_deg(0.f), h_fov_deg(0.f), v_fov_deg(0.f){};
  FOV(float y, float p, float h, float v) : yaw_deg(y), pitch_deg(p), h_fov_deg(h), v_fov_deg(v){};
  float yaw_deg;
  float pitch_deg;
  float h_fov_deg;
  float v_fov_deg;
};

#endif