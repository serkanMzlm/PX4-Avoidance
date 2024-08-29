#ifndef __TRANSFORM_FUNCTION_HPP__ 
#define __TRANSFORM_FUNCTION_HPP__

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

#define P2F(X) (1000 / X)
#define F2P(X) (X / 1000)

#define WARN_UNUSED __attribute__((warn_unused_result))

void eulerToQuaternion(float *euler, float *quaternion);
void quaternionToEuler(float *quaternion, float *euler);

/**
* @brief     wrappes the input angle in to plus minus PI space
* @param[in] angle to be wrapped  [rad]
* @returns   wrapped angle [rad]
**/
float WARN_UNUSED wrapAngleToPlusMinusPI(float angle);

/**
* @brief     wrappes the input angle in to plus minus 180 deg space
* @param[in] angle to be wrapped  [deg]
* @returns   wrapped angle [deg]
**/
float WARN_UNUSED wrapAngleToPlusMinus180(float angle);

#endif