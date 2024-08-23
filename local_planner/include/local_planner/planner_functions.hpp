#ifndef __PLANNER_FUNCTIONS_HPP__ 
#define __PLANNER_FUNCTIONS_HPP__

#include <pcl_conversions/pcl_conversions.h>

#include "avoidance/histogram.hpp"
#include "avoidance/common.hpp"

/**
* @brief      calculates a histogram from the current frame pointcloud around
*             the current vehicle position
* @param[out] polar_histogram, represents cropped_cloud
* @param[in]  cropped_cloud, current frame filtered pointcloud
* @param[in]  position, current vehicle position
**/
void generateNewHistogram(Histogram& polar_histogram, const Eigen::Vector3f& position,
                            const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud);

#endif