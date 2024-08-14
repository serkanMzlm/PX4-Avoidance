#ifndef __LOCAL_PLANNER_NODE_H__
#define __LOCAL_PLANNER_NODE_H__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/vehicle_trajectory_bezier.hpp>
#include <px4_msgs/msg/vehicle_trajectory_waypoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>

#include "local_planner_type.h"
#include "common.h"
#include "avoidance.h"
#include "local_planner.h"
#include "waypoint_generator.h"
#include "transform_buffer.h"

namespace avoidance
{
    typedef struct
    {
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr goal_topic;
    } Sub_t;

    typedef struct
    {
        rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr obs_distance;
        rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pose_setpoint;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command;
        rclcpp::Publisher<px4_msgs::msg::VehicleTrajectoryWaypoint>::SharedPtr trajector_waypoint;
    } Pub_t;

    typedef struct
    {
        float altitude;
    } Vehicle_s;

    typedef struct 
    {
        std::string topic_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        pcl::PointCloud<pcl::PointXYZ> untransformed_cloud_;
        bool received_;

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud_;
        bool transformed_;

        std::unique_ptr<std::mutex> camera_mutex_;
        std::unique_ptr<std::condition_variable> camera_cv_;

        bool transform_registered_ = false;
        std::thread transform_thread_;

        FOV fov_fcu_frame_;
    } cameraData;

    class LocalPlannerNode : public rclcpp::Node
    {
    public:
        LocalPlannerNode();
        virtual ~LocalPlannerNode();
        virtual void init();
        void updatePlannerInfo();
        void cmdLoopCallback();
        void calculateWaypoints(bool hover);
        void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void readParams();
        void initializeCameraSubscribers(std::vector<std::string> &camera_topics);
        void printPointInfo(double x, double y, double z);
        size_t numTransformedClouds();
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void pointCloudTransformThread(int index);
        void publishLaserScan() const;
        void threadFunction();
        void transformBufferThread();
        void clickedGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void updateGoalCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

        std::unique_ptr<LocalPlanner> local_planner_;
        std::unique_ptr<WaypointGenerator> wp_generator_;
        std::unique_ptr<AvoidanceNode> avoidance_node_;

        std::vector<cameraData> cameras_;
        std::atomic<bool> should_exit_{false};

        bool new_goal_ = false;
        bool armed_ = false;
        bool hover_;
        bool accept_goal_input_topic_;
        bool position_received_ = false;
        bool position_not_received_error_sent_ = false;
        bool is_land_waypoint_{false};
        bool is_takeoff_waypoint_{false};

        double spin_dt_;
        int index_ = 0;
        uint64_t px4_time = 0;

        std::condition_variable tf_buffer_cv_;
        std::condition_variable transformed_cloud_cv_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        Eigen::Vector3f velocity_;
        Eigen::Vector3f goal_position_;
        Eigen::Vector3f last_position_;
        Eigen::Vector3f newest_position_;
        Eigen::Vector3f prev_goal_position_;
        Eigen::Vector3f desired_velocity_;
        Eigen::Vector3f newest_waypoint_position_;
        Eigen::Vector3f last_waypoint_position_;
        Eigen::Vector3f last_adapted_waypoint_position_;
        Eigen::Vector3f newest_adapted_waypoint_position_;
        Eigen::Quaternionf newest_orientation_;

        tf_buffer::TransformBuffer tf_buffer_;
        NavigationState nav_state_ = NavigationState::none;

        std::mutex transformed_cloud_mutex_;
        std::mutex waypoints_mutex_;
        std::mutex running_mutex_;
        std::mutex buffered_transforms_mutex_;
        std::vector<std::pair<std::string, std::string>> buffered_transforms_;

        std::thread worker;
        std::thread worker_tf_listener;

        rclcpp::Time start_time_;
        rclcpp::Time last_wp_time_;
        rclcpp::Time t_status_sent_;

        Sub_t sub;
        Pub_t pub;
        rclcpp::TimerBase::SharedPtr timer_;
        Vehicle_s veh;

        void publishOffboardControlMode();
        void publishVehicleCommand(uint16_t command, float param1, float param2);
        void publishTrajectorySetpoint(float x, float y, float z, float yaw);
        void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
        void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
        void vehicleTrajectoryWaypointCallback(px4_msgs::msg::VehicleTrajectoryWaypoint msg);
        void vehicleUpdate();
    };
}

#endif