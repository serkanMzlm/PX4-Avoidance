#ifndef __LOCAL_PLANNER_NODE_HPP__
#define __LOCAL_PLANNER_NODE_HPP__

#include <mutex>
#include <atomic>
#include <string>
#include <thread>
#include <Eigen/Core>
#include <boost/bind.hpp>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> // fromROSMsg

#include <px4_msgs/msg/vehicle_status.hpp>

#include "common.h"
#include "avoidance_node.h"
#include "transform_buffer.h"
#include "avoidance_output.h"
#include "local_planner_visualization.h"
#include "local_planner.h"
#include "waypoint_generator.h"

using namespace avoidance;
using namespace px4_msgs::msg;

struct cameraData
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
};

class LocalPlannerNode : public rclcpp::Node
{
public:
    LocalPlannerNode();
    virtual ~LocalPlannerNode();
    void startNode();
    void readParams();
    virtual void onInit();
    void InitializeNode();
    void threadFunction();
    void cmdLoopCallback();
    void updatePlannerInfo();
    size_t numTransformedClouds();
    void publishLaserScan() const;
    void transformBufferThread();
    void pointCloudTransformThread(int index);
    void calculateWaypoints(bool hover);
    void printPointInfo(double x, double y, double z);
    void stateCallback(const VehicleStatus::UniquePtr msg);
    void initializeCameraSubscribers(std::vector<std::string>& camera_topics);
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void clickedGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void updateGoalCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover);

    std::unique_ptr<LocalPlanner> local_planner_;
    std::unique_ptr<WaypointGenerator> wp_generator_;
    std::unique_ptr<AvoidanceNode> avoidance_node_;
    LocalPlannerVisualization visualizer_;

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

    std::condition_variable tf_buffer_cv_;
    std::condition_variable transformed_cloud_cv_;

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

    rclcpp::Time start_time_;
    rclcpp::Time last_wp_time_;
    rclcpp::Time t_status_sent_;
};

#endif