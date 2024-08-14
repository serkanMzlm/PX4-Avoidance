#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "common.h"

namespace avoidance
{
  class AvoidanceNode
  {
  public:
    AvoidanceNode();
    ~AvoidanceNode() {}
    void px4ParamsInit();
    ModelParameters getPX4Parameters() const;
    float getMissionItemSpeed() const;

  private:
    ModelParameters px4_;
    bool position_received_;
    bool should_exit_;
    float mission_item_speed_;
    rclcpp::Logger avoidance_node_logger_ = rclcpp::get_logger("avoidance");
  };
}
#endif // AVOIDANCE_AVOIDANCE_NODE_H
