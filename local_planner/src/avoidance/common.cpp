#include "avoidance/common.hpp"

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
        if (prev_q.quaternion[i] + offset < current_q.quaternion[i] ||
            prev_q.quaternion[i] - offset > current_q.quaternion[i])
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
    if (isChangeOrientation(prev_state.orientation, current_state.orientation, offset))
    {
        return true;
    }

    return false;
}

bool pointInsideFOV(const std::vector<FOV> &fov_vec, const PolarPoint &p_pol)
{
    for (auto fov : fov_vec)
    {
        if (pointInsideFOV(fov, p_pol))
        {
            return true;
        }
    }
    return false;
}

bool pointInsideFOV(const FOV &fov, const PolarPoint &p_pol)
{
    return p_pol.z <= wrapAngleToPlusMinus180(fov.yaw_deg + fov.h_fov_deg / 2.f) &&
           p_pol.z >= wrapAngleToPlusMinus180(fov.yaw_deg - fov.h_fov_deg / 2.f) &&
           p_pol.e <= fov.pitch_deg + fov.v_fov_deg / 2.f &&
           p_pol.e >= fov.pitch_deg - fov.v_fov_deg / 2.f;
}

bool histogramIndexYawInsideFOV(const std::vector<FOV> &fov_vec, const int idx, Eigen::Vector3f position, float yaw_fcu_frame)
{
    PolarPoint pol_hist = histogramIndexToPolar(GRID_LENGTH_E / 2, idx, ALPHA_RES, 1.f);
    Eigen::Vector3f cart = polarHistogramToCartesian(pol_hist, position);
    PolarPoint pol_fcu = cartesianToPolarFCU(cart, position); // z down convention
    pol_fcu.z -= yaw_fcu_frame;                               // transform to fcu body frame
    PolarPoint pol_fcu_plus = pol_fcu;
    PolarPoint pol_fcu_minus = pol_fcu;
    pol_fcu_plus.z += ALPHA_RES / 2.f;
    pol_fcu_minus.z -= ALPHA_RES / 2.f;
    wrapPolar(pol_fcu_plus);
    wrapPolar(pol_fcu_minus);

    return (pointInsideFOV(fov_vec, pol_fcu_plus) || pointInsideFOV(fov_vec, pol_fcu_minus));
}

bool histogramIndexYawInsideFOV(const FOV &fov, const int idx, Eigen::Vector3f position, float yaw_fcu_frame)
{
    PolarPoint pol_hist = histogramIndexToPolar(GRID_LENGTH_E / 2, idx, ALPHA_RES, 1.f);
    Eigen::Vector3f cart = polarHistogramToCartesian(pol_hist, position);
    PolarPoint pol_fcu = cartesianToPolarFCU(cart, position); // z down convention
    pol_fcu.z -= yaw_fcu_frame;                               // transform to fcu body frame
    PolarPoint pol_fcu_plus = pol_fcu;
    PolarPoint pol_fcu_minus = pol_fcu;
    pol_fcu_plus.z += ALPHA_RES / 2.f;
    pol_fcu_minus.z -= ALPHA_RES / 2.f;
    wrapPolar(pol_fcu_plus);
    wrapPolar(pol_fcu_minus);

    return (pointInsideFOV(fov, pol_fcu_plus) || pointInsideFOV(fov, pol_fcu_minus));
}

bool pointInsideYawFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol) {
  for (auto fov : fov_vec) {
    if (pointInsideYawFOV(fov, p_pol)) {
      return true;
    }
  }
  return false;
}

bool pointInsideYawFOV(const FOV& fov, const PolarPoint& p_pol) {
  return p_pol.z <= wrapAngleToPlusMinus180(fov.yaw_deg + fov.h_fov_deg / 2.f) &&
         p_pol.z >= wrapAngleToPlusMinus180(fov.yaw_deg - fov.h_fov_deg / 2.f);
}