#include "local_planner/local_planner_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNode>("world"));
    rclcpp::shutdown();
    return 0;
}