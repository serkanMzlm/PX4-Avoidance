
#include <boost/algorithm/string.hpp>

#include "tree_node.h"
#include "local_planner.h"
#include "planner_functions.h"
#include "waypoint_generator.h"
#include "local_planner_node.hpp"

LocalPlannerNode::LocalPlannerNode() : Node("local_planner_node"),
                                       tf_buffer_(5.f),
                                       spin_dt_(0.1)
{
}

LocalPlannerNode::~LocalPlannerNode()
{
}

void LocalPlannerNode::onInit()
{
  RCLCPP_INFO(this->get_logger(), "Initializing nodelet...");
  InitializeNode();
}

void LocalPlannerNode::InitializeNode()
{
  local_planner_.reset(new LocalPlanner());
  wp_generator_.reset(new WaypointGenerator());
  avoidance_node_.reset(new AvoidanceNode());
  readParams();
}

void LocalPlannerNode::startNode()
{
  avoidance_node_->init();
}

void LocalPlannerNode::readParams()
{
  Eigen::Vector3d goal_d = goal_position_.cast<double>();

  this->declare_parameter<double>("goal_x_param", 0.0);
  this->declare_parameter<double>("goal_y_param", 0.0);
  this->declare_parameter<double>("goal_z_param", 0.0);
  this->declare_parameter<bool>("accept_goal_input_topic", false);

  goal_d.x() = this->get_parameter("goal_x_param").as_double();
  goal_d.y() = this->get_parameter("goal_y_param").as_double();
  goal_d.z() = this->get_parameter("goal_z_param").as_double();
  accept_goal_input_topic_ = this->get_parameter("accept_goal_input_topic").as_double();

  goal_position_ = goal_d.cast<float>();
  std::vector<std::string> camera_topics;
  initializeCameraSubscribers(camera_topics);
  new_goal_ = true;
}

void LocalPlannerNode::initializeCameraSubscribers(std::vector<std::string> &camera_topics)
{
  cameras_.resize(camera_topics.size());
  for (size_t i = 0; i < camera_topics.size(); i++)
  {
    cameras_[i].camera_mutex_.reset(new std::mutex);
    cameras_[i].camera_cv_.reset(new std::condition_variable);

    cameras_[i].received_ = false;
    cameras_[i].transformed_ = false;
    index_ = i;
    cameras_[i].pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(camera_topics[i], 100,
                                                                                           std::bind(&LocalPlannerNode::pointCloudCallback, this, std::placeholders::_1));
    cameras_[i].topic_ = camera_topics[i];
    cameras_[i].transform_thread_ = std::thread(&LocalPlannerNode::pointCloudTransformThread, this, i);
  }
}

void LocalPlannerNode::positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  last_position_ = newest_position_;
  newest_position_ = toEigen(msg->pose.position);
  newest_orientation_ = toEigen(msg->pose.orientation);

  position_received_ = true;
}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lck(*(cameras_[index_].camera_mutex_));

  auto timeSinceLast = [&]() -> rclcpp::Duration
  {
    rclcpp::Time lastCloudReceived = pcl_conversions::fromPCL(cameras_[index_].untransformed_cloud_.header.stamp);
    rclcpp::Time currentCloudReceived(msg->header.stamp.sec, msg->header.stamp.nanosec);
    return currentCloudReceived - lastCloudReceived;
  };

  if (cameras_[index_].received_ && timeSinceLast() < rclcpp::Duration::from_seconds(0.3))
  {
    return;
  }

  pcl::fromROSMsg(*msg, cameras_[index_].untransformed_cloud_);
  cameras_[index_].received_ = true;
  cameras_[index_].camera_cv_->notify_all();

  // this runs once at the beginning to get the transforms
  if (!cameras_[index_].transform_registered_)
  {
    std::lock_guard<std::mutex> tf_list_guard(buffered_transforms_mutex_);
    std::pair<std::string, std::string> transform_frames;
    transform_frames.first = msg->header.frame_id;
    transform_frames.second = "local_origin";
    buffered_transforms_.push_back(transform_frames);
    transform_frames.second = "fcu";
    buffered_transforms_.push_back(transform_frames);
    cameras_[index_].transform_registered_ = true;
  }
}

void LocalPlannerNode::pointCloudTransformThread(int index)
{
  // while (!should_exit_) {
  //   bool waiting_on_transform = false;
  //   bool waiting_on_cloud = false;
  //   {
  //     std::lock_guard<std::mutex> camera_lock(*(cameras_[index].camera_mutex_));

  //     if (cameras_[index].received_) {
  //       geometry_msgs::msg::TransformStamped cloud_transform;
  //       geometry_msgs::msg::TransformStamped fcu_transform;

  //       if (tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id, "local_origin",
  //                                   pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp),
  //                                   cloud_transform) &&
  //           tf_buffer_.getTransform(cameras_[index].untransformed_cloud_.header.frame_id, "fcu",
  //                                   pcl_conversions::fromPCL(cameras_[index].untransformed_cloud_.header.stamp),
  //                                   fcu_transform)) {
  //         // remove nan padding and compute fov
  //         pcl::PointCloud<pcl::PointXYZ> maxima = removeNaNAndGetMaxima(cameras_[index].untransformed_cloud_);

  //         // update point cloud FOV
  //         // pcl_ros::transformPointCloud(maxima, maxima, fcu_transform);
  //         updateFOVFromMaxima(cameras_[index].fov_fcu_frame_, maxima);

  //         // transform cloud to local_origin frame
  //         // pcl_ros::transformPointCloud(cameras_[index].untransformed_cloud_, cameras_[index].transformed_cloud_,
  //         //                              cloud_transform);
  //         cameras_[index].transformed_cloud_.header.frame_id = "local_origin";
  //         cameras_[index].transformed_cloud_.header.stamp = cameras_[index].untransformed_cloud_.header.stamp;

  //         cameras_[index].transformed_ = true;
  //         cameras_[index].received_ = false;
  //         waiting_on_cloud = true;
  //         std::lock_guard<std::mutex> lock(transformed_cloud_mutex_);
  //         transformed_cloud_cv_.notify_all();
  //       } else {
  //         waiting_on_transform = true;
  //       }
  //     } else {
  //       waiting_on_cloud = true;
  //     }
  //   }

  //   if (should_exit_) {
  //     break;
  //   }

  //   if (waiting_on_transform) {
  //     std::unique_lock<std::mutex> lck(buffered_transforms_mutex_);
  //     tf_buffer_cv_.wait_for(lck, std::chrono::milliseconds(5000));
  //   } else if (waiting_on_cloud) {
  //     std::unique_lock<std::mutex> lck(*(cameras_[index].camera_mutex_));
  //     cameras_[index].camera_cv_->wait_for(lck, std::chrono::milliseconds(5000));
  //   }
  // }
}

void LocalPlannerNode::publishLaserScan() const
{
  // if (!(local_planner_->px4_.param_cp_dist < 0)) {
  //   sensor_msgs::msg::LaserScan distance_data_to_fcu;
  //   local_planner_->getObstacleDistanceData(distance_data_to_fcu);

  //   // only send message if planner had a chance to fill it with valid data
  //   if (distance_data_to_fcu.angle_increment > 0.f) {
  //     mavros_obstacle_distance_pub_.publish(distance_data_to_fcu);
  //   }
  // }
}

size_t LocalPlannerNode::numTransformedClouds()
{
  size_t num_transformed_clouds = 0;
  for (size_t i = 0; i < cameras_.size(); i++)
  {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
    if (cameras_[i].transformed_)
      num_transformed_clouds++;
  }
  return num_transformed_clouds;
}

void LocalPlannerNode::updatePlannerInfo()
{
  // update the point cloud
  local_planner_->original_cloud_vector_.resize(cameras_.size());
  for (size_t i = 0; i < cameras_.size(); ++i)
  {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_[i].camera_mutex_));
    try
    {
      std::swap(local_planner_->original_cloud_vector_[i], cameras_[i].transformed_cloud_);
      cameras_[i].transformed_cloud_.clear();
      cameras_[i].transformed_ = false;
      local_planner_->setFOV(i, cameras_[i].fov_fcu_frame_);
      wp_generator_->setFOV(i, cameras_[i].fov_fcu_frame_);
    }
    catch (tf2::TransformException &ex)
    {
      // ROS_ERROR("Received an exception trying to transform a pointcloud: %s", ex.what());
    }
  }

  // update pose
  local_planner_->setState(newest_position_, velocity_, newest_orientation_);

  // update state
  local_planner_->currently_armed_ = armed_;

  // update goal
  if (new_goal_)
  {
    local_planner_->setGoal(goal_position_);
    local_planner_->setPreviousGoal(prev_goal_position_);
    new_goal_ = false;
  }

  // update last sent waypoint
  local_planner_->last_sent_waypoint_ = newest_waypoint_position_;

  // update the Firmware parameters
  local_planner_->px4_ = avoidance_node_->getPX4Parameters();
  local_planner_->mission_item_speed_ = avoidance_node_->getMissionItemSpeed();
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  velocity_ = toEigen(msg->twist.linear);
}

void LocalPlannerNode::stateCallback(const VehicleStatus::UniquePtr msg)
{
  armed_ = msg->arming_state == 2 ? true : false;

  if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_MISSION)
  {
    nav_state_ = NavigationState::mission;
  }
  else if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF)
  {
    nav_state_ = NavigationState::auto_takeoff;
  }
  else if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND)
  {
    nav_state_ = NavigationState::auto_land;
  }
  else if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_RTL)
  {
    nav_state_ = NavigationState::auto_rtl;
  }
  else if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LOITER)
  {
    nav_state_ = NavigationState::auto_loiter;
  }
  else if (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
  {
    nav_state_ = NavigationState::offboard;
  }
  else
  {
    nav_state_ = NavigationState::none;
  }
}

void LocalPlannerNode::cmdLoopCallback()
{
  std::lock_guard<std::mutex> lock(waypoints_mutex_);
  hover_ = false;

  // Process callbacks & wait for a position update
  rclcpp::Time start_query_position = rclcpp::Clock().now();

  while (!position_received_ && rclcpp::ok())
  {

    rclcpp::Duration since_query = rclcpp::Clock().now() - start_query_position;
    if (since_query > rclcpp::Duration(local_planner_->timeout_termination_, 0))
    {
      avoidance_node_->setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
      if (!position_not_received_error_sent_)
      {
        // clang-format off
        // ROS_WARN("\033[1;33m Planner abort: missing required data from FCU \n \033[0m");
        // ROS_WARN("----------------------------- Debugging Info -----------------------------");
        // ROS_WARN("Local planner has not received a position from FCU, check the following: ");
        // ROS_WARN("1. Check cables connecting PX4 autopilot with onboard computer");
        // ROS_WARN("2. Set PX4 parameter MAV_1_MODE to onbard or external vision");
        // ROS_WARN("3. Set correct fcu_url in local_planner launch file:");
        // ROS_WARN("   Example direct connection to serial port: /dev/ttyUSB0:921600");
        // ROS_WARN("   Example connection over mavlink router: udp://:14540@localhost:14557");
        // ROS_WARN("--------------------------------------------------------------------------");
        // clang-format on
        position_not_received_error_sent_ = true;
      }
    }
  }

  // Check if all information was received
  rclcpp::Time now = rclcpp::Clock().now();
  rclcpp::Duration since_last_cloud = now - last_wp_time_;
  rclcpp::Duration since_start = now - start_time_;

  checkFailsafe(since_last_cloud, since_start, hover_);

  // send waypoint
  if (avoidance_node_->getSystemStatus() == MAV_STATE::MAV_STATE_ACTIVE)
  {
    calculateWaypoints(hover_);
  }

  position_received_ = false;
}

void LocalPlannerNode::calculateWaypoints(bool hover)
{
  bool is_airborne = armed_ && (nav_state_ != NavigationState::none);

  wp_generator_->updateState(newest_position_, newest_orientation_, goal_position_, prev_goal_position_, velocity_,
                             hover, is_airborne, nav_state_, is_land_waypoint_, is_takeoff_waypoint_,
                             desired_velocity_);
  waypointResult result = wp_generator_->getWaypoints();

  Eigen::Vector3f closest_pt = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f deg60_pt = Eigen::Vector3f(NAN, NAN, NAN);
  wp_generator_->getOfftrackPointsForVisualization(closest_pt, deg60_pt);

  last_waypoint_position_ = newest_waypoint_position_;
  newest_waypoint_position_ = result.smoothed_goto_position;
  last_adapted_waypoint_position_ = newest_adapted_waypoint_position_;
  newest_adapted_waypoint_position_ = result.adapted_goto_position;

  // visualize waypoint topics
  visualizer_.visualizeWaypoints(result.goto_position, result.adapted_goto_position, result.smoothed_goto_position);
  visualizer_.publishPaths(last_position_, newest_position_, last_waypoint_position_, newest_waypoint_position_,
                           last_adapted_waypoint_position_, newest_adapted_waypoint_position_);
  visualizer_.publishCurrentSetpoint(toTwist(result.linear_velocity_wp, result.angular_velocity_wp),
                                     result.waypoint_type, newest_position_);

  visualizer_.publishOfftrackPoints(closest_pt, deg60_pt);

  // send waypoints to mavros
  // mavros_msgs::Trajectory obst_free_path = {};
  // transformToTrajectory(obst_free_path, toPoseStamped(result.position_wp, result.orientation_wp),
  //                       toTwist(result.linear_velocity_wp, result.angular_velocity_wp));
  // mavros_pos_setpoint_pub_.publish(toPoseStamped(result.position_wp, result.orientation_wp));

  // mavros_obstacle_free_path_pub_.publish(obst_free_path);
}

void LocalPlannerNode::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  printPointInfo(msg->point.x, msg->point.y, msg->point.z);
}

void LocalPlannerNode::clickedGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  new_goal_ = true;
  prev_goal_position_ = goal_position_;
  goal_position_ = toEigen(msg->pose.position);
  /* Selecting the goal from Rviz sets x and y. Get the z coordinate set in
   * the launch file */
  goal_position_.z() = local_planner_->getGoal().z();
}

void LocalPlannerNode::updateGoalCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  if (accept_goal_input_topic_ && msg->markers.size() > 0)
  {
    prev_goal_position_ = goal_position_;
    goal_position_ = toEigen(msg->markers[0].pose.position);
    new_goal_ = true;
  }
}

void LocalPlannerNode::printPointInfo(double x, double y, double z) 
{
  Eigen::Vector3f drone_pos = local_planner_->getPosition();
  int beta_z = floor((atan2(x - drone_pos.x(), y - drone_pos.y()) * 180.0 / M_PI));  //(-180. +180]
  int beta_e = floor((atan((z - drone_pos.z()) / (Eigen::Vector2f(x, y) - drone_pos.topRows<2>()).norm()) * 180.0 /
                      M_PI));  //(-90.+90)

  beta_z = beta_z + (ALPHA_RES - beta_z % ALPHA_RES);  //[-170,+190]
  beta_e = beta_e + (ALPHA_RES - beta_e % ALPHA_RES);  //[-80,+90]

  RCLCPP_INFO(this->get_logger(), "----- Point: %f %f %f -----\n", x, y, z);
  RCLCPP_INFO(this->get_logger(), "Elevation %d Azimuth %d \n", beta_e, beta_z);
  RCLCPP_INFO(this->get_logger(), "-------------------------------------------- \n");
}


// void LocalPlannerNode::fcuInputGoalCallback(const mavros_msgs::Trajectory& msg) {
//   bool update =
//       ((avoidance::toEigen(msg.point_2.position) - avoidance::toEigen(goal_mission_item_msg_.pose.position)).norm() >
//        0.01) ||
//       !std::isfinite(goal_position_(0)) || !std::isfinite(goal_position_(1));
//   if ((msg.point_valid[0] == true) && update) {
//     new_goal_ = true;
//     prev_goal_position_ = goal_position_;
//     goal_position_ = toEigen(msg.point_1.position);
//     desired_velocity_ = toEigen(msg.point_1.velocity);
//     is_land_waypoint_ = (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_LAND));
//     is_takeoff_waypoint_ = (msg.command[0] == static_cast<int>(MavCommand::MAV_CMD_NAV_TAKEOFF));
//   }
//   if (msg.point_valid[1] == true) {
//     goal_mission_item_msg_.pose.position = msg.point_2.position;
//     if (msg.command[1] == UINT16_MAX) {
//       goal_position_ = toEigen(msg.point_2.position);
//       desired_velocity_ << NAN, NAN, NAN;
//     }
//     desired_yaw_setpoint_ = msg.point_2.yaw;
//     desired_yaw_speed_setpoint_ = msg.point_2.yaw_rate;
//   }
// }

// void LocalPlannerNode::distanceSensorCallback(const mavros_msgs::Altitude& msg) {
//   if (!std::isnan(msg.bottom_clearance)) {
//     ground_distance_msg_ = msg;
//   }
// }

void LocalPlannerNode::transformBufferThread() {
  // grab transforms from tf and store them into the buffer
  while (!should_exit_) {
    {
      std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
      for (auto const& frame_pair : buffered_transforms_) {
        geometry_msgs::msg::TransformStamped transform;

        // if (tf_listener_->canTransform(frame_pair.second, frame_pair.first, rclcpp::Time(0))) {
        //   try {
        //     tf_listener_->lookupTransform(frame_pair.second, frame_pair.first, rclcpp::Time(0), transform);
        //     tf_buffer_.insertTransform(frame_pair.first, frame_pair.second, transform);
        //   } catch (tf2::TransformException& ex) {
        //     RCLCPP_ERROR(this->get_logger(), "Received an exception trying to get transform: %s", ex.what());
        //   }
        // }
      }
      tf_buffer_cv_.notify_all();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void LocalPlannerNode::threadFunction() {
  while (!should_exit_) {
    rclcpp::Time start_time = rclcpp::Clock().now();

    while ((cameras_.size() == 0 || cameras_.size() != numTransformedClouds()) && !should_exit_) {
      std::unique_lock<std::mutex> lock(transformed_cloud_mutex_);
      transformed_cloud_cv_.wait_for(lock, std::chrono::milliseconds(5000));
    }

    if (should_exit_) break;

    {
      std::lock_guard<std::mutex> guard(running_mutex_);
      updatePlannerInfo();
      local_planner_->runPlanner();

      visualizer_.visualizePlannerData(*(local_planner_.get()), newest_waypoint_position_,
                                       newest_adapted_waypoint_position_, newest_position_, newest_orientation_);
      publishLaserScan();

      std::lock_guard<std::mutex> lock(waypoints_mutex_);
      wp_generator_->setPlannerInfo(local_planner_->getAvoidanceOutput());
      last_wp_time_ = rclcpp::Clock().now();
    }

    if (should_exit_) break;

    rclcpp::Duration loop_time = last_wp_time_ - start_time;
    rclcpp::Duration required_delay = rclcpp::Duration(spin_dt_, 0) - loop_time;
    if (required_delay > rclcpp::Duration(0, 0)) {
      // required_delay.sleep();
    }
  }
}

void LocalPlannerNode::checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover) {
  avoidance_node_->checkFailsafe(since_last_cloud, since_start, hover);
}

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerNode>());

  rclcpp::shutdown();
  return 0;
}
