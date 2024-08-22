#ifndef __COMMON_HPP__ 
#define __COMMON_HPP__ 

#include "local_planner/local_planner_types.hpp"

bool isChangePose(Vec3_t prev_pos, Vec3_t current_pos, float offset = 0.0);
bool isChangeOrientation(Orientation_t prev_q, Orientation_t current_q, float offset = 0.0);
bool isChangeState(State_t prev_state, State_t current_state, float offset = 0.0);

#endif