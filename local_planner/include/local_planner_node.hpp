#ifndef __LOCAL_PLANNER_NODE_HPP__
#define __LOCAL_PLANNER_NODE_HPP__

#include "local_planner_types.hpp"
#include "math_tools/transform_functions.hpp"

#define NUMBER_OF_FUNCTIONS 0b00000111 // Number of functions to check for is ready

#define POINTCLOUD_FLAG (1 << 1)       // 0001
#define VEHICLE_STATUS_FLAG (1 << 0)   // 0010
#define VEHICLE_ODOM_FLAG (1 << 2)     // 0100

typedef struct
{
    rclcpp::Subscription<pointCloud2Msg>::SharedPtr point_cloud;
    rclcpp::Subscription<vehicleOdomMsg>::SharedPtr vehicle_odom;
    rclcpp::Subscription<vehicleStatusMsg>::SharedPtr vehicle_status;
} Sub_t;

typedef struct
{
    rclcpp::Publisher<navPathMsg>::SharedPtr vehicle_path;
    rclcpp::Publisher<obstacleDistanceMsg>::SharedPtr obs_distance;
    rclcpp::Publisher<vehicleCommandMsg>::SharedPtr vehicle_command;
    rclcpp::Publisher<trajectorySetpointMsg>::SharedPtr trajectory_setpoint;
    rclcpp::Publisher<offboardControlModeMsg>::SharedPtr offboard_control_mode;
    rclcpp::Publisher<vehicleTrajectoryWaypointMsg>::SharedPtr trajector_waypoint;
} Pub_t;

typedef struct
{
    rclcpp::TimerBase::SharedPtr main;
    rclcpp::TimerBase::SharedPtr cmd_loop;
    rclcpp::TimerBase::SharedPtr visual;
} TimeBase_t;

typedef struct
{
    pcl::PCLPointCloud2 pc;
    pcl::PointCloud<pcl::PointXYZ> pc_xyz;
    navPathMsg path;
    vehicleStatusMsg status;
    std::string frame_name;
} Data_t;

class LocalPlannerNode : public rclcpp::Node
{
private:
    State_t state = {0};
    State_t setpoint = {0};
    State_t prev_state = {0};

    Pub_t pub;
    Sub_t sub;
    TimeBase_t time_base;
    Data_t data;

    uint8_t is_ready = 0x00;

    tfStamped tf_stamped;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_vehicle;

public:
    LocalPlannerNode();
    ~LocalPlannerNode() = default;

    void vehicleOdomCallback(const vehicleOdomMsg::UniquePtr msg);
    void vehicleStatusCallback(const vehicleStatusMsg::UniquePtr msg);
    void pointCloudCallback(const pointCloud2Msg::SharedPtr);
    void visualizationCallback();

    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    bool setArmedState(ArmState armed);
    bool setArm();
    bool setDisarm();

    void initTopic();
    bool isReady();

private:
    void isReadyUpdate(uint8_t flag);
};

#endif