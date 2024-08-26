#ifndef __LOCAL_PLANNER_TYPES_HPP__
#define __LOCAL_PLANNER_TYPES_HPP__

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

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

enum waypoint_choice
{
    hover,
    tryPath,
    direct,
    reachHeight
};

struct avoidanceOutput
{
    float cruise_velocity;       // mission cruise velocity
    rclcpp::Time last_path_time; // finish built time for the VFH+* tree

    std::vector<Eigen::Vector3f> path_node_positions; // array of tree nodes  position, each node is the
                                                      // minimum cost node for each tree depth level
};

struct costParameters
{
    float yaw_cost_param = 0.5f;
    float pitch_cost_param = 3.f;
    float velocity_cost_param = 1.5f;
    float obstacle_cost_param = 5.0f;
};

struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : e(e_), z(z_), r(r_){};
  PolarPoint() : e(0.0f), z(0.0f), r(0.0f){};
  float e;
  float z;
  float r;
};


struct candidateDirection
{
    float cost;
    float elevation_angle;
    float azimuth_angle;

    candidateDirection(float c, float e, float z) : cost(c), elevation_angle(e), azimuth_angle(z) {};

    bool operator<(const candidateDirection &y) const { return cost < y.cost; }

    bool operator>(const candidateDirection &y) const { return cost > y.cost; }

    PolarPoint toPolar(float r) const { return PolarPoint(elevation_angle, azimuth_angle, r); }
};

#endif