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
           p_pol.e <= fov.pitch_deg + fov.v_fov_deg / 2.f && p_pol.e >= fov.pitch_deg - fov.v_fov_deg / 2.f;
}

PolarPoint cartesianToPolarHistogram(const Eigen::Vector3f &pos, const Eigen::Vector3f &origin)
{
    return cartesianToPolarHistogram(pos.x(), pos.y(), pos.z(), origin);
}

PolarPoint cartesianToPolarHistogram(float x, float y, float z, const Eigen::Vector3f &pos)
{
    PolarPoint p_pol(0.0f, 0.0f, 0.0f);
    float den = (Eigen::Vector2f(x, y) - pos.topRows<2>()).norm();
    p_pol.e = std::atan2(z - pos.z(), den) * RAD_TO_DEG;
    p_pol.z = std::atan2(x - pos.x(), y - pos.y()) * RAD_TO_DEG;
    p_pol.r = sqrt((x - pos.x()) * (x - pos.x()) + (y - pos.y()) * (y - pos.y()) + (z - pos.z()) * (z - pos.z()));
    return p_pol;
}

Eigen::Vector2i polarToHistogramIndex(const PolarPoint& p_pol, int res) {
  Eigen::Vector2i ev2(0, 0);
  PolarPoint p_wrapped = p_pol;
  wrapPolar(p_wrapped);
  // elevation angle to y-axis histogram index
  // maps elevation -90° to bin 0 and +90° to the highest bin (N-1)
  ev2.y() = static_cast<int>(floor(p_wrapped.e / res + 90.0f / res));
  // azimuth angle to x-axis histogram index
  // maps elevation -180° to bin 0 and +180° to the highest bin (N-1)
  ev2.x() = static_cast<int>(floor(p_wrapped.z / res + 180.0f / res));

  // clamp due to floating point errros
  if (ev2.x() >= 360 / res) ev2.x() = 360 / res - 1;
  if (ev2.x() < 0) ev2.x() = 0;
  if (ev2.y() >= 180 / res) ev2.y() = 180 / res - 1;
  if (ev2.y() < 0) ev2.y() = 0;

  return ev2;
}

void wrapPolar(PolarPoint& p_pol) {
  // first wrap the angles to +-180 degrees
  p_pol.e = wrapAngleToPlusMinus180(p_pol.e);
  p_pol.z = wrapAngleToPlusMinus180(p_pol.z);

  // elevation valid [-90,90)
  // when abs(elevation) > 90, wrap elevation angle
  // azimuth changes 180 if it wraps

  bool wrapped = false;
  if (p_pol.e > 90.0f) {
    p_pol.e = 180.0f - p_pol.e;
    wrapped = true;
  } else if (p_pol.e < -90.0f) {
    p_pol.e = -(180.0f + p_pol.e);
    wrapped = true;
  }
  if (wrapped) {
    if (p_pol.z < 0.f)
      p_pol.z += 180.f;
    else
      p_pol.z -= 180.f;
  }
}

bool histogramIndexYawInsideFOV(const std::vector<FOV>& fov_vec, const int idx, Eigen::Vector3f position,
                                float yaw_fcu_frame) {
  PolarPoint pol_hist = histogramIndexToPolar(GRID_LENGTH_E / 2, idx, ALPHA_RES, 1.f);
  Eigen::Vector3f cart = polarHistogramToCartesian(pol_hist, position);
  PolarPoint pol_fcu = cartesianToPolarFCU(cart, position);  // z down convention
  pol_fcu.z -= yaw_fcu_frame;                                // transform to fcu body frame
  PolarPoint pol_fcu_plus = pol_fcu;
  PolarPoint pol_fcu_minus = pol_fcu;
  pol_fcu_plus.z += ALPHA_RES / 2.f;
  pol_fcu_minus.z -= ALPHA_RES / 2.f;
  wrapPolar(pol_fcu_plus);
  wrapPolar(pol_fcu_minus);

  return (pointInsideFOV(fov_vec, pol_fcu_plus) || pointInsideFOV(fov_vec, pol_fcu_minus));
}

bool histogramIndexYawInsideFOV(const FOV& fov, const int idx, Eigen::Vector3f position, float yaw_fcu_frame) {
  PolarPoint pol_hist = histogramIndexToPolar(GRID_LENGTH_E / 2, idx, ALPHA_RES, 1.f);
  Eigen::Vector3f cart = polarHistogramToCartesian(pol_hist, position);
  PolarPoint pol_fcu = cartesianToPolarFCU(cart, position);  // z down convention
  pol_fcu.z -= yaw_fcu_frame;                                // transform to fcu body frame
  PolarPoint pol_fcu_plus = pol_fcu;
  PolarPoint pol_fcu_minus = pol_fcu;
  pol_fcu_plus.z += ALPHA_RES / 2.f;
  pol_fcu_minus.z -= ALPHA_RES / 2.f;
  wrapPolar(pol_fcu_plus);
  wrapPolar(pol_fcu_minus);

  return (pointInsideFOV(fov, pol_fcu_plus) || pointInsideFOV(fov, pol_fcu_minus));
}

PolarPoint cartesianToPolarFCU(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin) {
  PolarPoint p = cartesianToPolarHistogram(pos, origin);
  p.z = -p.z + 90.0f;
  p.e = -p.e;
  wrapPolar(p);
  return p;
}

PolarPoint cartesianToPolarFCU(const pcl::PointXYZ& p) {
  return cartesianToPolarFCU(Eigen::Vector3f(p.x, p.y, p.z), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
}

PolarPoint histogramIndexToPolar(int e, int z, int res, float radius) {
  // ALPHA_RES%2=0 as per definition, see histogram.h
  PolarPoint p_pol(static_cast<float>(e * res + res / 2 - 90), static_cast<float>(z * res + res / 2 - 180), radius);
  return p_pol;
}

Eigen::Vector3f polarHistogramToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos) {
  Eigen::Vector3f p;
  p.x() = pos.x() + p_pol.r * std::cos(p_pol.e * DEG_TO_RAD) * std::sin(p_pol.z * DEG_TO_RAD);
  p.y() = pos.y() + p_pol.r * std::cos(p_pol.e * DEG_TO_RAD) * std::cos(p_pol.z * DEG_TO_RAD);
  p.z() = pos.z() + p_pol.r * std::sin(p_pol.e * DEG_TO_RAD);

  return p;
}