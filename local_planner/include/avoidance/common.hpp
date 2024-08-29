#ifndef __COMMON_HPP__ 
#define __COMMON_HPP__ 

#include "local_planner_types.hpp"
#include "math_tools/transform_functions.hpp"

bool isChangePose(Vec3_t prev_pos, Vec3_t current_pos, float offset = 0.0);
bool isChangeOrientation(Orientation_t prev_q, Orientation_t current_q, float offset = 0.0);
bool isChangeState(State_t prev_state, State_t current_state, float offset = 0.0);

/**
* @brief      determines whether point is inside FOV
* @param[in]  vector of FOV structs defining current field of view
* @param[in]  p_pol, polar representation of the point in question
* @return     whether point is inside the FOV
**/
bool pointInsideFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol);
bool pointInsideFOV(const FOV& fov, const PolarPoint& p_pol);

/**
* @brief      determines whether a histogram cell lies inside the horizontal FOV
*             cell is considered inside if at least one edge lies inside
* @param[in]  vector of FOV structs defining current field of view
* @param[in]  idx, histogram cell column index
* @param[in]  position, current position
* @param[in]  yaw_fcu_frame, yaw orientation of the vehicle in global fcu frame
* @return     whether point is inside the FOV
**/
bool histogramIndexYawInsideFOV(const std::vector<FOV>& fov_vec, const int idx, Eigen::Vector3f position, float yaw_fcu_frame);
bool histogramIndexYawInsideFOV(const FOV& fov, const int idx, Eigen::Vector3f position, float yaw_fcu_frame);

/**
* @brief      determines whether point is inside the Yaw of the FOV
* @param[in]  vector of FOV structs defining current field of view
* @param[in]  p_pol, polar representation of the point in question
* @return     whether point is inside the FOV
**/
bool pointInsideYawFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol);
bool pointInsideYawFOV(const FOV& fov, const PolarPoint& p_pol);

#endif