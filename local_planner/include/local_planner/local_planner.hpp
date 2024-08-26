#ifndef __LOCAL_PLANNER_HPP__
#define __LOCAL_PLANNER_HPP__

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "avoidance/common.hpp"
#include "avoidance/histogram.hpp"
#include "local_planner/local_planner_node_types.hpp"

class LocalPlanner
{
private:
    DistanceSensorFeatures_t sensor;
    std::vector<FOV> fov_fcu_frame_;
    obstacleDistanceMsg obs_distance;

    Histogram polar_histogram_ = Histogram(ALPHA_RES);
    Histogram to_fcu_histogram_ = Histogram(ALPHA_RES);

    Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_ = Eigen::Vector3f::Zero();

    std::vector<int> closed_set_;

public:
    LocalPlanner();
    void setup();
    /**
     * @brief fills message to send histogram to the FCU
     **/
    void updateObstacleDistanceMsg(Histogram hist);
};

#endif