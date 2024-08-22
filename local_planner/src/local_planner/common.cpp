#include "local_planner/common.hpp"
#include <rclcpp/rclcpp.hpp>

bool isChangePose(Vec3_t prev_pos, Vec3_t current_pos, float offset)
{   
    for(int i = 0; i < 3; i++)
    {
        if(prev_pos.data[i] + offset < current_pos.data[i] || 
                    prev_pos.data[i] - offset > current_pos.data[i])
        {
            return true;
        }
    }
    return false;
}

bool isChangeOrientation(Orientation_t prev_q, Orientation_t current_q, float offset)
{
    for(int i = 0; i < 4; i++)
    {
        if(prev_q.q[i] + offset < current_q.q[i] || 
                    prev_q.q[i] - offset > current_q.q[i])
        {
            return true;
        }
    }
    return false;
}

bool isChangeState(State_t prev_state, State_t current_state, float offset)
{
    if(isChangePose(prev_state.position, current_state.position, offset))
    {
        return true;
    }
    if(isChangeOrientation(prev_state.quaternion, current_state.quaternion, offset))
    {
        return true;
    }

    return false;
}
