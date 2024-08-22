#ifndef __LOCAL_PLANNER_NODE_HPP__
#define __LOCAL_PLANNER_NODE_HPP__

#include "geometry_tools/geometry_tools.hpp"
#include "math_tools/math_operations.hpp"
#include "local_planner/local_planner_node_types.hpp"
#include "local_planner/common.hpp"

class LocalPlannerNode: public rclcpp::Node
{
private:
    Sub_t sub;
    Pub_t pub;
    TimeBase_t time_base;

    State_t state = {0};
    State_t prev_state = {0};
    State_t setpoint = {0};

    navPathMsg vehicle_path;
    tfStamped tf_stamped;
    vehicleStatusMsg vehicle_status;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_vehicle;

    std::string frame_id_name;
    bool is_ready[READY_FLAG_SIZE] = {false};
    bool system_is_ready = false;
public:
    LocalPlannerNode(std::string frame_name = "world");
    ~LocalPlannerNode() = default;
    void initTopic();
    bool isReady();

    void vehicleOdomCallback(const vehicleOdomMsg::UniquePtr msg);
    void vehicleStatusCallback(const vehicleStatusMsg::UniquePtr msg);
    void pointCloudCallback(const pointCloud2Msg::SharedPtr);
    void visualizationCallback();

    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    bool setArmedState(ArmState armed);
};

#endif