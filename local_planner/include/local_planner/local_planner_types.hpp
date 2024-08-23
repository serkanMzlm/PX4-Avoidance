#ifndef __LOCAL_PLANNER_TYPES_HPP__
#define __LOCAL_PLANNER_TYPES_HPP__

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float data[3];
} Vec3_t;

typedef union
{
    struct
    {
        float x;
        float y;
        float z;
        float w;
    };
    float q[4];
} Orientation_t;

typedef union
{
    struct
    {
        float roll;
        float pitch;
        float yaw;
    };
    float data[3];
} Attitude_t;

typedef struct
{
    Vec3_t position;
    Vec3_t velocity;
    Vec3_t acceleration;
    Attitude_t attitude;
    Orientation_t quaternion;
} State_t;

enum class ArmState
{
    ARM,
    DISARM
};

typedef enum ReadyFlag
{
    VEH_ODOM_F,
    VEH_STATUS_F,
    READY_FLAG_SIZE
};

typedef struct DistanceSensorFeatures_t
{
    float min_range;
    float max_range;
    float smoothing_margin_deg;
    float max_point_age_s;
    float yaw_fcu_frame_deg;
    float pitch_fcu_frame_deg;
};

#endif