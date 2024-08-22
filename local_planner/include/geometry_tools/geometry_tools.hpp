#ifndef __GEOMETRY_TOOLS_HPP__
#define __GEOMETRY_TOOLS_HPP__

#include <cmath>

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180.0f)
#endif

#ifndef RAD2DEG
#define RAD2DEG (180.0f / M_PI)
#endif

void eulerToQuaternion(float *euler, float *quaternion);
void quaternionToEuler(float *quaternion, float *euler);

float wrapAngleToPlusMinusPI(float angle);
float wrapAngleToPlusMinus180(float angle);


#endif