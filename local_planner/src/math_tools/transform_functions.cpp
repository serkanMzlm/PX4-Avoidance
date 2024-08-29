#include <cmath>

#include "math_tools/transform_functions.hpp"

void eulerToQuaternion(float *euler, float *quaternion)
{
    float roll = euler[0];
    float pitch = euler[1];
    double yaw = euler[2];

    // Calculate the half angles
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quaternion[0] = sr * cp * cy - cr * sp * sy; // x
    quaternion[1] = cr * sp * cy + sr * cp * sy; // y
    quaternion[2] = cr * cp * sy - sr * sp * cy; // z
    quaternion[3] = cr * cp * cy + sr * sp * sy; // w
}

void quaternionToEuler(float *quaternion, float *euler)
{
    float x = quaternion[0];
    float y = quaternion[1];
    float z = quaternion[2];
    float w = quaternion[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0 * (w * x + y * z);
    float cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        euler[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler[1] = std::asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0 * (w * z + x * y);
    float cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
}

float wrapAngleToPlusMinusPI(float angle)
{
    return angle - 2.0f * M_PI * std::floor(angle / (2.0f * M_PI) + 0.5f);
}

float wrapAngleToPlusMinus180(float angle)
{
    return angle - 360.f * std::floor(angle / 360.f + 0.5f);
}