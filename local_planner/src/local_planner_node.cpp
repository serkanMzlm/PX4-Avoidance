#include "local_planner_node.h"

using namespace std::placeholders;
namespace avoidance
{

    LocalPlannerNode::LocalPlannerNode() : spin_dt_(0.1), Node("local_planner_node"), tf_buffer_(5.f)
    {
        readParams();
        init();
    }

    LocalPlannerNode::~LocalPlannerNode()
    {
        should_exit_ = true;
        {
            std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
            tf_buffer_cv_.notify_all();
        }

        {
            std::lock_guard<std::mutex> guard(transformed_cloud_mutex_);
            transformed_cloud_cv_.notify_all();
        }

        for (size_t i = 0; i < 1; ++i)
        {
            {
                std::lock_guard<std::mutex> guard(*cameras_.camera_mutex_);
                cameras_.camera_cv_->notify_all();
            }
            if (cameras_.transform_thread_.joinable())
                cameras_.transform_thread_.join();
        }

        if (worker.joinable())
            worker.join();

        if (worker_tf_listener.joinable())
            worker_tf_listener.join();
    }

    void LocalPlannerNode::init()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Node...");
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        sub.odom = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
                                                                             std::bind(&LocalPlannerNode::odomCallback, this, _1));
        sub.vehicle_status = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos,
                                                                                     std::bind(&LocalPlannerNode::vehicleStatusCallback, this, _1));
        sub.clicked_point = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10,
                                                                                        std::bind(&LocalPlannerNode::clickedPointCallback, this, _1));
        sub.pose_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
                                                                                   std::bind(&LocalPlannerNode::clickedGoalCallback, this, _1));
        sub.goal_topic = this->create_subscription<visualization_msgs::msg::MarkerArray>("input/goal_position", 10,
                                                                                         std::bind(&LocalPlannerNode::updateGoalCallback, this, _1));

        pub.obs_distance = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);
        pub.trajector_waypoint = this->create_publisher<px4_msgs::msg::VehicleTrajectoryWaypoint>("/fmu/in/vehicle_trajectory_waypoint", 10);
        pub.offboard_control_mode = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&LocalPlannerNode::vehicleUpdate, this));

        local_planner_.reset(new LocalPlanner());
        wp_generator_.reset(new WaypointGenerator());
        avoidance_node_.reset(new AvoidanceNode());

        worker = std::thread(&LocalPlannerNode::threadFunction, this);
        worker_tf_listener = std::thread(&LocalPlannerNode::transformBufferThread, this);
    }

    void LocalPlannerNode::updatePlannerInfo()
    {
        // update the point cloud
        local_planner_->original_cloud_vector_.resize(1);
        for (size_t i = 0; i < 1; ++i)
        {
            std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_.camera_mutex_));
            try
            {
                std::swap(local_planner_->original_cloud_vector_[i], cameras_.transformed_cloud_);
                cameras_.transformed_cloud_.clear();
                cameras_.transformed_ = false;
                local_planner_->setFOV(i, cameras_.fov_fcu_frame_);
                wp_generator_->setFOV(i, cameras_.fov_fcu_frame_);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Received an exception trying to transform a pointcloud: %s", ex.what());
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

    void LocalPlannerNode::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        veh.altitude = -msg->position[2];
        last_position_ = newest_position_;

        // msg -> NED  = ENU
        newest_position_ = toEigen(msg->position[1], msg->position[0], -msg->position[2]);
        newest_orientation_ = toEigen(msg->q[1], msg->q[0], -msg->q[2], msg->q[3]);
        velocity_ = toEigen(msg->velocity[1], msg->velocity[0], -msg->velocity[2]);
        position_received_ = true;
        px4_time = msg->timestamp;
    }

    void LocalPlannerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        armed_ = msg->arming_state == 2 ? 1 : 0;
        switch (msg->nav_state)
        {
        case 3:
            nav_state_ = NavigationState::mission;
            break;
        case 4:
            nav_state_ = NavigationState::auto_loiter;
            break;
        case 5:
            nav_state_ = NavigationState::auto_rtl;
            break;
        case 14:
            nav_state_ = NavigationState::offboard;
            break;
        case 17:
            nav_state_ = NavigationState::auto_takeoff;
            break;
        case 18:
            nav_state_ = NavigationState::auto_land;
            break;

        default:
            nav_state_ = NavigationState::none;
            break;
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
                if (!position_not_received_error_sent_)
                {
                    // clang-format off
                    RCLCPP_WARN(this->get_logger(), "\033[1;33m Planner abort: missing required data from FCU \n \033[0m");
                    RCLCPP_WARN(this->get_logger(), "----------------------------- Debugging Info -----------------------------");
                    RCLCPP_WARN(this->get_logger(), "Local planner has not received a position from FCU, check the following: ");
                    RCLCPP_WARN(this->get_logger(), "1. Check cables connecting PX4 autopilot with onboard computer");
                    RCLCPP_WARN(this->get_logger(), "2. Set PX4 parameter MAV_1_MODE to onbard or external vision");
                    RCLCPP_WARN(this->get_logger(), "3. Set correct fcu_url in local_planner launch file:");
                    RCLCPP_WARN(this->get_logger(), "   Example direct connection to serial port: /dev/ttyUSB0:921600");
                    RCLCPP_WARN(this->get_logger(), "   Example connection over mavlink router: udp://:14540@localhost:14557");
                    RCLCPP_WARN(this->get_logger(), "--------------------------------------------------------------------------");
                    // clang-format on
                    position_not_received_error_sent_ = true;
                }
            }
        }

        // Check if all information was received
        rclcpp::Time now = rclcpp::Clock().now();
        rclcpp::Duration since_last_cloud = now - last_wp_time_;
        rclcpp::Duration since_start = now - start_time_;

        calculateWaypoints(hover_);
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

        px4_msgs::msg::VehicleTrajectoryWaypoint obst_free_path;
        transformToTrajectory(obst_free_path, toPoseStamped(result.position_wp, result.orientation_wp),
                              toTwist(result.linear_velocity_wp, result.angular_velocity_wp));
        vehicleTrajectoryWaypointCallback(obst_free_path);
    }

    void LocalPlannerNode::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        printPointInfo(msg->point.x, msg->point.y, msg->point.z);
    }

    void LocalPlannerNode::readParams()
    {
        Eigen::Vector3d goal_d = goal_position_.cast<double>();

        this->declare_parameter<double>("goal_x_param", 15.0);
        this->declare_parameter<double>("goal_y_param", 15.0);
        this->declare_parameter<double>("goal_z_param", 4.0);
        this->declare_parameter<bool>("accept_goal_input_topic", false);

        goal_d.x() = this->get_parameter("goal_x_param").as_double();
        goal_d.y() = this->get_parameter("goal_y_param").as_double();
        goal_d.z() = this->get_parameter("goal_z_param").as_double();
        accept_goal_input_topic_ = this->get_parameter("accept_goal_input_topic").as_bool();

        goal_position_ = goal_d.cast<float>();
        std::string camera_topics = "/realsense_d435i/points";
        initializeCameraSubscribers(camera_topics);
        new_goal_ = true;
    }

    void LocalPlannerNode::initializeCameraSubscribers(std::string &camera_topics)
    {
        cameras_.camera_mutex_.reset(new std::mutex);
        cameras_.camera_cv_.reset(new std::condition_variable);

        cameras_.received_ = false;
        cameras_.transformed_ = false;
        index_ = 0;
        cameras_.pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            camera_topics, 100, std::bind(&LocalPlannerNode::pointCloudCallback, this, std::placeholders::_1));
        cameras_.topic_ = camera_topics;
        cameras_.transform_thread_ = std::thread(&LocalPlannerNode::pointCloudTransformThread, this, 0);
    }

    void LocalPlannerNode::printPointInfo(double x, double y, double z)
    {
        Eigen::Vector3f drone_pos = local_planner_->getPosition();
        int beta_z = floor((atan2(x - drone_pos.x(), y - drone_pos.y()) * 180.0 / M_PI)); //(-180. +180]
        int beta_e = floor((atan((z - drone_pos.z()) / (Eigen::Vector2f(x, y) - drone_pos.topRows<2>()).norm()) * 180.0 /
                            M_PI)); //(-90.+90)

        beta_z = beta_z + (ALPHA_RES - beta_z % ALPHA_RES); //[-170,+190]
        beta_e = beta_e + (ALPHA_RES - beta_e % ALPHA_RES); //[-80,+90]

        RCLCPP_INFO(this->get_logger(), "----- Point: %f %f %f -----\n", x, y, z);
        RCLCPP_INFO(this->get_logger(), "Elevation %d Azimuth %d \n", beta_e, beta_z);
        RCLCPP_INFO(this->get_logger(), "-------------------------------------------- \n");
    }

    size_t LocalPlannerNode::numTransformedClouds()
    {
        size_t num_transformed_clouds = 0;
        for (size_t i = 0; i < 1; i++)
        {
            std::lock_guard<std::mutex> transformed_cloud_guard(*(cameras_.camera_mutex_));
            if (cameras_.transformed_)
                num_transformed_clouds++;
        }
        return num_transformed_clouds;
    }

    void LocalPlannerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lck(*(cameras_.camera_mutex_));

        auto timeSinceLast = [&]() -> rclcpp::Duration
        {
            rclcpp::Time lastCloudReceived = pcl_conversions::fromPCL(cameras_.untransformed_cloud_.header.stamp);
            rclcpp::Time currentCloudReceived(msg->header.stamp.sec, msg->header.stamp.nanosec);
            return currentCloudReceived - lastCloudReceived;
        };

        if (cameras_.received_ && timeSinceLast() < rclcpp::Duration::from_seconds(0.3))
        {
            return;
        }

        pcl::fromROSMsg(*msg, cameras_.untransformed_cloud_);
        cameras_.received_ = true;
        cameras_.camera_cv_->notify_all();

        // this runs once at the beginning to get the transforms
        if (!cameras_.transform_registered_)
        {
            std::lock_guard<std::mutex> tf_list_guard(buffered_transforms_mutex_);
            std::pair<std::string, std::string> transform_frames;
            transform_frames.first = msg->header.frame_id;
            transform_frames.second = "local_origin";
            buffered_transforms_.push_back(transform_frames);
            transform_frames.second = "fcu";
            buffered_transforms_.push_back(transform_frames);
            cameras_.transform_registered_ = true;
        }
    }

    void LocalPlannerNode::pointCloudTransformThread(int index)
    {
        while (!should_exit_)
        {
            bool waiting_on_transform = false;
            bool waiting_on_cloud = false;
            {
                std::lock_guard<std::mutex> camera_lock(*(cameras_.camera_mutex_));

                if (cameras_.received_)
                {
                    geometry_msgs::msg::TransformStamped cloud_transform;
                    geometry_msgs::msg::TransformStamped fcu_transform;
                    rclcpp::Time time_stamp_1(cameras_.untransformed_cloud_.header.stamp, RCL_ROS_TIME);
                    rclcpp::Time time_stamp_2(cameras_.untransformed_cloud_.header.stamp, RCL_ROS_TIME);
                    if (tf_buffer_.getTransform(cameras_.untransformed_cloud_.header.frame_id, "local_origin", time_stamp_1,  cloud_transform) &&
                        tf_buffer_.getTransform(cameras_.untransformed_cloud_.header.frame_id, "fcu", time_stamp_2, fcu_transform))
                    {
                        // remove nan padding and compute fov
                        pcl::PointCloud<pcl::PointXYZ> maxima = removeNaNAndGetMaxima(cameras_.untransformed_cloud_);

                        // update point cloud FOV
                        // pcl_ros::transformPointCloud(maxima, maxima, fcu_transform);
                        updateFOVFromMaxima(cameras_.fov_fcu_frame_, maxima);

                        // transform cloud to local_origin frame
                        // pcl_ros::transformPointCloud(cameras_.untransformed_cloud_, cameras_.transformed_cloud_,
                        //                              cloud_transform);
                        cameras_.transformed_cloud_.header.frame_id = "local_origin";
                        cameras_.transformed_cloud_.header.stamp = cameras_.untransformed_cloud_.header.stamp;

                        cameras_.transformed_ = true;
                        cameras_.received_ = false;
                        waiting_on_cloud = true;
                        std::lock_guard<std::mutex> lock(transformed_cloud_mutex_);
                        transformed_cloud_cv_.notify_all();
                    }
                    else
                    {
                        waiting_on_transform = true;
                    }
                }
                else
                {
                    waiting_on_cloud = true;
                }
            }

            if (should_exit_)
            {
                break;
            }

            if (waiting_on_transform)
            {
                std::unique_lock<std::mutex> lck(buffered_transforms_mutex_);
                tf_buffer_cv_.wait_for(lck, std::chrono::milliseconds(5000));
            }
            else if (waiting_on_cloud)
            {
                std::unique_lock<std::mutex> lck(*(cameras_.camera_mutex_));
                cameras_.camera_cv_->wait_for(lck, std::chrono::milliseconds(5000));
            }
        }
    }

    void LocalPlannerNode::transformBufferThread()
    {
        // grab transforms from tf and store them into the buffer
        while (!should_exit_)
        {
            {
                std::lock_guard<std::mutex> guard(buffered_transforms_mutex_);
                for (auto const &frame_pair : buffered_transforms_)
                {
                    geometry_msgs::msg::TransformStamped transform;
                    try
                    {
                        // tf_listener_->lookupTransform(frame_pair.second, frame_pair.first, tf2::TimePointZero);
                        tf_buffer_.insertTransform(frame_pair.first, frame_pair.second, transform);
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Received an exception trying to get transform: %s", ex.what());
                    }
                }
                tf_buffer_cv_.notify_all();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void LocalPlannerNode::publishLaserScan() const
    {
        if (!(local_planner_->px4_.param_cp_dist < 0))
        {
            sensor_msgs::msg::LaserScan distance_data_to_fcu;
            px4_msgs::msg::ObstacleDistance obs_dist;

            obs_dist.frame = obs_dist.MAV_FRAME_BODY_FRD;
            obs_dist.sensor_type = obs_dist.MAV_DISTANCE_SENSOR_LASER;

            local_planner_->getObstacleDistanceData(distance_data_to_fcu);

            obs_dist.increment = 2;
            obs_dist.min_distance = distance_data_to_fcu.range_min;
            obs_dist.max_distance = distance_data_to_fcu.range_max;
            obs_dist.angle_offset = 0;
            obs_dist.timestamp = px4_time;
            pub.obs_distance->publish(obs_dist);
        }
    }

    void LocalPlannerNode::threadFunction()
    {
        while (!should_exit_)
        {
            rclcpp::Time start_time = rclcpp::Clock().now();

            while (!should_exit_)
            {
                std::unique_lock<std::mutex> lock(transformed_cloud_mutex_);
                transformed_cloud_cv_.wait_for(lock, std::chrono::milliseconds(5000));
            }

            if (should_exit_)
                break;

            {
                std::lock_guard<std::mutex> guard(running_mutex_);
                updatePlannerInfo();
                local_planner_->runPlanner();

                publishLaserScan();
                RCLCPP_INFO(this->get_logger(), "Publish");
                std::lock_guard<std::mutex> lock(waypoints_mutex_);
                wp_generator_->setPlannerInfo(local_planner_->getAvoidanceOutput());
                last_wp_time_ = rclcpp::Clock().now();
            }

            if (should_exit_)
                break;

            rclcpp::Duration loop_time = last_wp_time_ - start_time;
            rclcpp::Duration required_delay = rclcpp::Duration::from_seconds(spin_dt_) - loop_time;
            if (required_delay > rclcpp::Duration::from_seconds(0))
            {
                rclcpp::sleep_for(std::chrono::nanoseconds(required_delay.nanoseconds()));
            }
        }
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

    void LocalPlannerNode::publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub.offboard_control_mode->publish(msg);
    }

    void LocalPlannerNode::publishTrajectorySetpoint(float x, float y, float z, float yaw)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position[0] = y;
        msg.position[1] = x;
        msg.position[2] = z;
        // msg.velocity[0] = setpoint.velocity.x;
        // msg.velocity[1] = setpoint.velocity.y;
        // msg.velocity[2] = setpoint.velocity.z;
        msg.yaw = yaw; // [-PI:PI]
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub.trajectory_setpoint->publish(msg);
    }

    void LocalPlannerNode::publishVehicleCommand(uint16_t command, float param1, float param2)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub.vehicle_command->publish(msg);
    }

    void LocalPlannerNode::vehicleTrajectoryWaypointCallback(px4_msgs::msg::VehicleTrajectoryWaypoint msg)
    {
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        RCLCPP_INFO_STREAM(this->get_logger(), "data: " << msg.waypoints[0].position[0]);
        pub.trajector_waypoint->publish(msg);
    }

    void LocalPlannerNode::vehicleUpdate()
    {
        // px4_msgs::msg::VehicleTrajectoryWaypoint msg;
        // msg.waypoints[0].position[0] = 2.0;
        // msg.waypoints[0].position[1] = 2.0;
        // msg.waypoints[0].position[2] = 2.0;

        // msg.waypoints[1].position[0] = 4.0;
        // msg.waypoints[1].position[1] = 2.0;
        // msg.waypoints[1].position[2] = 2.0;

        // msg.waypoints[2].position[0] = 6.0;
        // msg.waypoints[2].position[1] = 2.0;
        // msg.waypoints[2].position[2] = 2.0;

        // msg.waypoints[3].position[0] = 8.0;
        // msg.waypoints[3].position[1] = 2.0;
        // msg.waypoints[3].position[2] = 2.0;

        // msg.waypoints[4].position[0] = 10.0;
        // msg.waypoints[4].position[1] = 2.0;
        // msg.waypoints[4].position[2] = 2.0;
        publishOffboardControlMode();
        // vehicleTrajectoryWaypointCallback(msg);
    }

} // namespace avoidance
