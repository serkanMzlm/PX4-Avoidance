#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#include "local_planner/local_planner_types.hpp"

struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : e(e_), z(z_), r(r_){};
  PolarPoint() : e(0.0f), z(0.0f), r(0.0f){};
  float e;
  float z;
  float r;
};

/**
* Struct defining the Field of View of the sensor. This is defined by
* the current azimuth and elevation angles and the horizontal and vertical
* field of view of the sensor
*/
struct FOV {
  FOV() : yaw_deg(0.f), pitch_deg(0.f), h_fov_deg(0.f), v_fov_deg(0.f){};
  FOV(float y, float p, float h, float v) : yaw_deg(y), pitch_deg(p), h_fov_deg(h), v_fov_deg(v){};
  float yaw_deg;
  float pitch_deg;
  float h_fov_deg;
  float v_fov_deg;
};

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

#endif