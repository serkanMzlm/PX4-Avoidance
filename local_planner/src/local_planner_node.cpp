#include "local_planner_node.hpp"
using namespace std::placeholders;

LocalPlannerNode::LocalPlannerNode() : Node("local_planner_node")
{
    data.frame_name = "world";
    initTopic();
}

void LocalPlannerNode::vehicleOdomCallback(const vehicleOdomMsg::UniquePtr msg)
{
    state.position.x = msg->position[0];
    state.position.y = msg->position[1];
    state.position.z = msg->position[2];

    state.velocity.x = msg->velocity[0];
    state.velocity.y = msg->velocity[1];
    state.velocity.z = msg->velocity[2];

    state.orientation.w = msg->q[3];
    state.orientation.x = msg->q[0];
    state.orientation.y = msg->q[1];
    state.orientation.z = msg->q[2];

    quaternionToEuler(state.orientation.quaternion, state.orientation.euler);
    isReadyUpdate(VEHICLE_ODOM_FLAG);
}

void LocalPlannerNode::vehicleStatusCallback(const vehicleStatusMsg::UniquePtr msg)
{
    data.status.failsafe = msg->failsafe;
    data.status.nav_state = msg->nav_state;
    data.status.arming_state = msg->arming_state;
    data.status.nav_state_user_intention = msg->nav_state_user_intention;
    data.status.failure_detector_status = msg->failure_detector_status;
    isReadyUpdate(VEHICLE_STATUS_FLAG);
}

void LocalPlannerNode::pointCloudCallback(const pointCloud2Msg::SharedPtr msg)
{
    pcl_conversions::toPCL(*msg, data.pc);
    pcl::fromPCLPointCloud2(data.pc, data.pc_xyz);
    isReadyUpdate(POINTCLOUD_FLAG);
}

void LocalPlannerNode::visualizationCallback()
{
    if (!isReady())
    {
        RCLCPP_INFO_STREAM(this->get_logger(),"is_ready: " << static_cast<unsigned int>(is_ready));
        RCLCPP_WARN(this->get_logger(), "The system is not ready");
        return;
    }

    tf_stamped.header.stamp = this->get_clock()->now();
    tf_stamped.header.frame_id = data.frame_name;
    tf_stamped.child_frame_id = "base_link";
    tf_stamped.transform.rotation.x = state.orientation.quaternion[0];
    tf_stamped.transform.rotation.y = state.orientation.quaternion[1];
    tf_stamped.transform.rotation.z = state.orientation.quaternion[2];
    tf_stamped.transform.rotation.w = state.orientation.quaternion[3];
    tf_stamped.transform.translation.x = state.position.x;
    tf_stamped.transform.translation.y = state.position.y;
    tf_stamped.transform.translation.z = -state.position.z;

    tf_vehicle->sendTransform(tf_stamped);
    // if (!isChangePose(prev_state.position, state.position, 0.1))
    // {
    //     return;
    // }

    prev_state = state;

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = tf_stamped.header;
    pose_stamped.pose.position.x = tf_stamped.transform.translation.x;
    pose_stamped.pose.position.y = tf_stamped.transform.translation.y;
    pose_stamped.pose.position.z = tf_stamped.transform.translation.z;

    data.path.header = tf_stamped.header;
    data.path.poses.push_back(pose_stamped);
    pub.vehicle_path->publish(data.path);
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
    msg.yaw = state.orientation.yaw; // [-PI:PI]
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
        return setArm();
    }

    return setDisarm();
}

bool LocalPlannerNode::setArm()
{
    if (data.status.arming_state == 2)
    {
        RCLCPP_WARN(this->get_logger(), "The vehicle is already armed...");
        return false;
    }
    publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
    return true;
}

bool LocalPlannerNode::setDisarm()
{
    if (data.status.arming_state == 1)
    {
        RCLCPP_WARN(this->get_logger(), "The vehicle is already disarmed...");
        return false;
    }
    publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    publishVehicleCommand(vehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
    return true;
}

void LocalPlannerNode::initTopic()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub.point_cloud = this->create_subscription<pointCloud2Msg>("/realsense_d435i/points", 100, std::bind(&LocalPlannerNode::pointCloudCallback, this, _1));
    sub.vehicle_odom = this->create_subscription<vehicleOdomMsg>("/fmu/out/vehicle_odometry", qos, std::bind(&LocalPlannerNode::vehicleOdomCallback, this, _1));
    sub.vehicle_status = this->create_subscription<vehicleStatusMsg>("/fmu/out/vehicle_status", qos, std::bind(&LocalPlannerNode::vehicleStatusCallback, this, _1));

    pub.vehicle_path = this->create_publisher<navPathMsg>("vehicle_path", 10);
    pub.obs_distance = this->create_publisher<obstacleDistanceMsg>("/fmu/in/obstacle_distance", 10);
    pub.trajectory_setpoint = this->create_publisher<trajectorySetpointMsg>("/fmu/in/trajectory_setpoint", 10);
    pub.offboard_control_mode = this->create_publisher<offboardControlModeMsg>("/fmu/in/offboard_control_mode", 10);
    pub.trajector_waypoint = this->create_publisher<vehicleTrajectoryWaypointMsg>("/fmu/in/vehicle_trajectory_waypoint", 100);

    time_base.visual = this->create_wall_timer(std::chrono::milliseconds(P2F(10)), std::bind(&LocalPlannerNode::visualizationCallback, this));
    // time_base.main = this->create_wall_timer(std::chrono::milliseconds(P2F(50)), std::bind(&LocalPlannerNode::mainFunction, this));

    tf_vehicle = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

bool LocalPlannerNode::isReady()
{
    return is_ready == NUMBER_OF_FUNCTIONS;
}

void LocalPlannerNode::isReadyUpdate(uint8_t flag)
{
    if (isReady())
    {
        return;
    }

    is_ready |= flag;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}