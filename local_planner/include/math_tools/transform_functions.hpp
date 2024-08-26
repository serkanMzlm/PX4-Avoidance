#ifndef __TRANSFORM_FUNCTION_HPP__ 
#define __TRANSFORM_FUNCTION_HPP__

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

#define P2F(X) (1000 / X)
#define F2P(X) (X / 1000)

void eulerToQuaternion(float *euler, float *quaternion);
void quaternionToEuler(float *quaternion, float *euler);

#endif