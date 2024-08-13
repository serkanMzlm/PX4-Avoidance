#include "local_planner_node.h"
using namespace avoidance;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}