#include "local_planner/local_planner.hpp"

LocalPlanner::LocalPlanner()
{
    setup();
}

void LocalPlanner::setup()
{
    sensor.max_range = 6.0f;
    sensor.min_range = 0.6f;
    sensor.smoothing_margin_deg = 30.0f;
    sensor.max_point_age_s = 10.0f;
    sensor.pitch_fcu_frame_deg = 0.0f;
    sensor.yaw_fcu_frame_deg = 0.0f;

    obs_distance.frame = obs_distance.MAV_FRAME_BODY_FRD;
    obs_distance.sensor_type = obs_distance.MAV_DISTANCE_SENSOR_LASER;
    obs_distance.min_distance = sensor.min_range * 100; // cm
    obs_distance.max_distance = sensor.min_range * 100; // cm
}

void LocalPlanner::updateObstacleDistanceMsg(Histogram hist)
{
    obstacleDistanceMsg msg = {};
    msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
    for (int i = 0; i < GRID_LENGTH_Z; i++)
    {
        int j = (i + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
        float dist = hist.get_dist(0, j);
        if (histogramIndexYawInsideFOV(fov_fcu_frame_, j, position_, sensor.yaw_fcu_frame_deg))
        {
            msg.distances[j] = (dist > sensor.min_range ? dist : sensor.max_range + 0.01f);
        }
        else
        {
            msg.distances[j] = sensor.max_range + 0.01f;
        }
    }
}