#include <chrono>
#include <thread>

#include "local_planner/local_planner_node.hpp"

using namespace std::placeholders;

LocalPlannerNode::LocalPlannerNode(std::string frame_name) : Node("local_planner_node"),
                                                             frame_id_name(frame_name)
{
    initTopic();
}

void LocalPlannerNode::initTopic()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub.vehicle_odom = this->create_subscription<vehicleOdomMsg>("/fmu/out/vehicle_odometry", qos,
                                                                 std::bind(&LocalPlannerNode::vehicleOdomCallback, this, _1));
    sub.vehicle_status = this->create_subscription<vehicleStatusMsg>("/fmu/out/vehicle_status", qos,
                                                                     std::bind(&LocalPlannerNode::vehicleStatusCallback, this, _1));
    sub.point_cloud = this->create_subscription<pointCloud2Msg>("/realsense_d435i/points", 100, 
                                                                    std::bind(&LocalPlannerNode::pointCloudCallback, this, _1));                                                                     

    pub.obs_distance = this->create_publisher<obstacleDistanceMsg>("/fmu/in/obstacle_distance", 10);
    pub.trajectory_setpoint = this->create_publisher<trajectorySetpointMsg>("/fmu/in/trajectory_setpoint", 10);
    pub.offboard_control_mode = this->create_publisher<offboardControlModeMsg>("/fmu/in/offboard_control_mode", 10);
    pub.trajector_waypoint = this->create_publisher<vehicleTrajectoryWaypointMsg>("/fmu/in/vehicle_trajectory_waypoint", 100);
    pub.vehicle_path = this->create_publisher<navPathMsg>("vehicle_path", 10);

    tf_vehicle = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    time_base.visual = this->create_wall_timer(std::chrono::milliseconds(P2F(10)),
                                               std::bind(&LocalPlannerNode::visualizationCallback, this));
}

bool LocalPlannerNode::isReady()
{
    if (system_is_ready)
    {
        return true;
    }

    for (int i = 0; i < sizeof(is_ready) / sizeof(is_ready[0]); i++)
    {
        if (!is_ready[i])
        {
            return false;
        }
    }

    system_is_ready = true;
    return true;
}

/////////////////////////////////////////////////
//                  Callback                   //
/////////////////////////////////////////////////
void LocalPlannerNode::vehicleOdomCallback(const vehicleOdomMsg::UniquePtr msg)
{
    state.position.x = msg->position[0];
    state.position.y = msg->position[1];
    state.position.z = msg->position[2];

    state.velocity.x = msg->velocity[0];
    state.velocity.y = msg->velocity[1];
    state.velocity.z = msg->velocity[2];

    state.quaternion.w = msg->q[3];
    state.quaternion.x = msg->q[0];
    state.quaternion.y = msg->q[1];
    state.quaternion.z = msg->q[2];

    quaternionToEuler(state.quaternion.q, state.attitude.data);
    is_ready[VEH_ODOM_F] = true;
}

void LocalPlannerNode::vehicleStatusCallback(const vehicleStatusMsg::UniquePtr msg)
{
    vehicle_status.failsafe = msg->failsafe;
    vehicle_status.nav_state = msg->nav_state;
    vehicle_status.arming_state = msg->arming_state;
    vehicle_status.nav_state_user_intention = msg->nav_state_user_intention;
    vehicle_status.failure_detector_status = msg->failure_detector_status;
    is_ready[VEH_STATUS_F] = true;
}

void LocalPlannerNode::pointCloudCallback(const pointCloud2Msg::SharedPtr msg)
{
}

void LocalPlannerNode::visualizationCallback()
{
    if (!isReady())
    {
        RCLCPP_WARN(this->get_logger(), "The system is not ready");
        return;
    }

    tf_stamped.header.stamp = this->get_clock()->now();
    tf_stamped.header.frame_id = frame_id_name;
    tf_stamped.child_frame_id = "base_link";
    tf_stamped.transform.rotation.x = state.quaternion.q[0];
    tf_stamped.transform.rotation.y = state.quaternion.q[1];
    tf_stamped.transform.rotation.z = state.quaternion.q[2];
    tf_stamped.transform.rotation.w = state.quaternion.q[3];

    if (isChangePose(prev_state.position, state.position, 0.1))
    {
        tf_stamped.transform.translation.x = state.position.x;
        tf_stamped.transform.translation.y = state.position.y;
        tf_stamped.transform.translation.z = -state.position.z;

        for (int i = 0; i < 3; i++)
        {
            prev_state.position.data[i] = state.position.data[i];
            prev_state.velocity.data[i] = state.velocity.data[i];
            prev_state.acceleration.data[i] = state.acceleration.data[i];
            prev_state.attitude.data[i] = state.attitude.data[i];
        }
        for (int i = 0; i < 4; i++)
        {
            prev_state.quaternion.q[i] = state.quaternion.q[i];
        }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = tf_stamped.header;
        pose_stamped.pose.position.x = tf_stamped.transform.translation.x;
        pose_stamped.pose.position.y = tf_stamped.transform.translation.y;
        pose_stamped.pose.position.z = tf_stamped.transform.translation.z;

        vehicle_path.header = tf_stamped.header;
        vehicle_path.poses.push_back(pose_stamped);
        pub.vehicle_path->publish(vehicle_path);
    }

    tf_vehicle->sendTransform(tf_stamped);
}

/////////////////////////////////////////////////
//                 PX4 Control                 //
/////////////////////////////////////////////////
void LocalPlannerNode::publishOffboardControlMode()
{
    offboardControlModeMsg msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub.offboard_control_mode->publish(msg);
}

void LocalPlannerNode::publishTrajectorySetpoint()
{
    trajectorySetpointMsg msg{};
    msg.position = {state.position.x, state.position.y, state.position.z};
    msg.yaw = state.attitude.yaw; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub.trajectory_setpoint->publish(msg);
}

void LocalPlannerNode::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    vehicleCommandMsg msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    pub.vehicle_command->publish(msg);
}

bool LocalPlannerNode::setArmedState(ArmState armed)
{
    if (armed == ArmState::ARM)
    {
        if (vehicle_status.arming_state == 2)
        {
            RCLCPP_WARN(this->get_logger(), "The vehicle is already armed...");
            return false;
        }
        publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command send");
    }
    else
    {
        if (vehicle_status.arming_state == 1)
        {
            RCLCPP_WARN(this->get_logger(), "The vehicle is already disarmed...");
            return false;
        }
        publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command send");
    }

    return true;
}