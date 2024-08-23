#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "local_planner/local_planner_types.hpp"
#include "geometry_tools/geometry_tools.hpp"

#include "avoidance/histogram.hpp"

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

/**
* @brief     Compute polar vector in histogram convention between two cartesian
*            points
* @param[in] position Position of the location to which to compute the bearing
*            angles to.
* @param[in] origin Origin from which to compute the bearing vectors.
* @details   For a point given in cartesian x/y coordinates this is the
*            angle in degrees from the positive y-axis in (-180, 180].
*
* @returns   azimuth Angle in float degrees from the positive y-axis (-180, 180]
*            and elevation angle degrees (-90, 90]
**/
PolarPoint cartesianToPolarHistogram(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin);
PolarPoint cartesianToPolarHistogram(float x, float y, float z, const Eigen::Vector3f& pos);

/**
* @brief     compute polar point to histogram index
* @param[in] p_pol with elevation, azimuth angle and radius
* @param[in] res resolution of the histogram in degrees
* @param[out]vector with x()=azimuth and y()=elevation
* @note      If there is an invalid input, the output index will be 0
**/
Eigen::Vector2i polarToHistogramIndex(const PolarPoint& p_pol, int res);

/**
* @brief     support function for polarToHistogramIndex
*            when abs(elevation) > 90, wrap elevation angle into valid
*            region and azimuth angle changes for +/-180 deg each time
* @param[in/out]p_pol Polar point with elevation angle [-90,90) and
*            azimuth angle [-180,180)
**/
void wrapPolar(PolarPoint& p_pol);

/**
* @brief      determines whether a histogram cell lies inside the horizontal FOV
*             cell is considered inside if at least one edge lies inside
* @param[in]  vector of FOV structs defining current field of view
* @param[in]  idx, histogram cell column index
* @param[in]  position, current position
* @param[in]  yaw_fcu_frame, yaw orientation of the vehicle in global fcu frame
* @return     whether point is inside the FOV
**/
bool histogramIndexYawInsideFOV(const std::vector<FOV>& fov_vec, const int idx, Eigen::Vector3f position,
                                float yaw_fcu_frame);
bool histogramIndexYawInsideFOV(const FOV& fov, const int idx, Eigen::Vector3f position, float yaw_fcu_frame);

/**
* @brief     Compute the polar vector in FCU convention between two cartesian
*            points
* @param[in] endpoint of the polar vector
* @param[in] origin of the polar vector
* @returns   polar point in FCU convention that points from the given origin to
*            the given point
* @warning   the output adheres to the FCU convention: positive yaw is measured
*            CCW from the positive x-axis, and positve pitch is measured CCW
*            from the positve x-axis. (Positive pitch is pitching forward)
* @note      An overloaded function taking a pcl::PointXYZ assumes the origin
*            (0, 0, 0)
**/
PolarPoint cartesianToPolarFCU(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin);
PolarPoint cartesianToPolarFCU(const pcl::PointXYZ& p);

/**
* @brief     compute point in the histogram to a polar point
* @param[in] e evelation index in the histogram
* @param[in] z azimuth index in the histogram
* @param[in] res resolution of the histogram
* @param[in] radius of the polar point
* @param[out]polar point with elevation angle, azimuth angle and radius
**/
PolarPoint histogramIndexToPolar(int e, int z, int res, float radius);

/**
* @brief     Converts a polar point in histogram convention to a cartesian point
*            and add it to a cartesian position
* @param[in] polar point in histogram convention to be converted
* @param[in] cartesian position, from which to convert the polar point
* @returns   point in cartesian CS
* @warning   The histogram convention means zero-azimuth is the positive y axis
*            with increasing azimuth in CW direction, while the elevation angle
*            increases for "upward looking" (contrary to pitch in FCU!)
**/
Eigen::Vector3f polarHistogramToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos);

inline Eigen::Vector3f toEigen(const geometry_msgs::msg::Point& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(float x, float y, float z) {
  Eigen::Vector3f ev3(x, y, z);
  return ev3;
}

inline Eigen::Quaternionf toEigen(float x, float y, float z, float w) {
  Eigen::Quaternionf eqf;
  eqf.x() = x;
  eqf.y() = y;
  eqf.z() = z;
  eqf.w() = w;
  return eqf;
}

inline Eigen::Vector3f toEigen(const geometry_msgs::msg::Vector3& v3) {
  Eigen::Vector3f ev3(v3.x, v3.y, v3.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const pcl::PointXYZ& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const pcl::PointXYZI& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Quaternionf toEigen(const geometry_msgs::msg::Quaternion& gmq) {
  Eigen::Quaternionf eqf;
  eqf.x() = gmq.x;
  eqf.y() = gmq.y;
  eqf.z() = gmq.z;
  eqf.w() = gmq.w;
  return eqf;
}

#endif