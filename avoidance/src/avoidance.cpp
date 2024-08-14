#include "avoidance.h"

#include <thread>

using namespace std::chrono_literals;

namespace avoidance
{
  AvoidanceNode::AvoidanceNode(): position_received_(true),  should_exit_(false)
  {

    this->px4ParamsInit();
    mission_item_speed_ = px4_.param_mpc_xy_cruise;
  } 

  void AvoidanceNode::px4ParamsInit()
  {
    px4_.param_mpc_acc_down_max = 3.0f;
    px4_.param_mpc_acc_hor = 3.0f;
    px4_.param_acc_up_max = 4.0f;
    px4_.param_mpc_jerk_min = 8.0f;
    px4_.param_mpc_jerk_max = 8.0f;
    px4_.param_mpc_land_speed = 0.7f;
    px4_.param_mpc_tko_speed = 1.5f;
    px4_.param_mpc_xy_cruise = 5.0f;
    px4_.param_mpc_z_vel_max_up = 1.0f;
    px4_.param_cp_dist = -1.0f;
    px4_.param_nav_acc_rad = 10.0;
  }

  ModelParameters AvoidanceNode::getPX4Parameters() const
  {
    return px4_;
  }

  float AvoidanceNode::getMissionItemSpeed() const
  {
    return mission_item_speed_;
  }

}
