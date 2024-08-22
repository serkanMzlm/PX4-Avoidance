#include "avoidance/common.hpp"
#include "geometry_tools/geometry_tools.hpp"

bool isChangePose(Vec3_t prev_pos, Vec3_t current_pos, float offset)
{
    for (int i = 0; i < 3; i++)
    {
        if (prev_pos.data[i] + offset < current_pos.data[i] ||
            prev_pos.data[i] - offset > current_pos.data[i])
        {
            return true;
        }
    }
    return false;
}

bool isChangeOrientation(Orientation_t prev_q, Orientation_t current_q, float offset)
{
    for (int i = 0; i < 4; i++)
    {
        if (prev_q.q[i] + offset < current_q.q[i] ||
            prev_q.q[i] - offset > current_q.q[i])
        {
            return true;
        }
    }
    return false;
}

bool isChangeState(State_t prev_state, State_t current_state, float offset)
{
    if (isChangePose(prev_state.position, current_state.position, offset))
    {
        return true;
    }
    if (isChangeOrientation(prev_state.quaternion, current_state.quaternion, offset))
    {
        return true;
    }

    return false;
}

bool pointInsideFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol) {
  for (auto fov : fov_vec) {
    if (pointInsideFOV(fov, p_pol)) {
      return true;
    }
  }
  return false;
}

bool pointInsideFOV(const FOV& fov, const PolarPoint& p_pol) {
  return p_pol.z <= wrapAngleToPlusMinus180(fov.yaw_deg + fov.h_fov_deg / 2.f) &&
         p_pol.z >= wrapAngleToPlusMinus180(fov.yaw_deg - fov.h_fov_deg / 2.f) &&
         p_pol.e <= fov.pitch_deg + fov.v_fov_deg / 2.f && p_pol.e >= fov.pitch_deg - fov.v_fov_deg / 2.f;
}
