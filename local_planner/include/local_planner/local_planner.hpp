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
    pcl::PointCloud<pcl::PointXYZI> final_cloud_;

    Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f prev_goal_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f closest_pt_ = Eigen::Vector3f::Zero();

public:
    LocalPlanner();
    void setup();
    /**
     * @brief fills message to send histogram to the FCU
     **/
    void updateObstacleDistanceMsg(Histogram hist);

    /**
     * @brief     setter method for vehicle position, orientation and velocity
     * @param[in] pos, vehicle position coming from the FCU
     * @param[in] vel, vehicle velocity in the FCU frame
     * @param[in] q, vehicle orientation message coming from the FCU
     **/
    void setState(const Eigen::Vector3f &pos, const Eigen::Vector3f &vel, const Eigen::Quaternionf &q);
};

#endif