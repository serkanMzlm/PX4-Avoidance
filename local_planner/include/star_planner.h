#ifndef __STAR_PLANNER_H__
#define __STAR_PLANNER_H__

#include "histogram.h"
#include "local_planner_type.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/transform_listener.h>

#include <px4_msgs/msg/vehicle_trajectory_bezier.hpp>
#include <px4_msgs/msg/vehicle_trajectory_waypoint.hpp>

namespace avoidance
{
    class TreeNode;

    class StarPlanner
    {
        int children_per_node_ = 1;
        int n_expanded_nodes_ = 5;
        float tree_node_distance_ = 1.0f;
        float max_path_length_ = 4.f;
        float smoothing_margin_degrees_ = 30.f;
        float tree_heuristic_weight_ = 10.0f;
        float max_sensor_range_ = 15.f;
        float min_sensor_range_ = 0.2f;

        pcl::PointCloud<pcl::PointXYZI> cloud_;

        Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
        Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
        Eigen::Vector3f velocity_ = Eigen::Vector3f(NAN, NAN, NAN);
        Eigen::Vector3f closest_pt_ = Eigen::Vector3f(NAN, NAN, NAN);

        costParameters cost_params_;

        public:
            float treeHeuristicFunction(int node_number) const;
            std::vector<Eigen::Vector3f> path_node_positions_;
            std::vector<int> closed_set_;
            std::vector<TreeNode> tree_;

            StarPlanner();
            ~StarPlanner() = default;  
            void setParams(costParameters cost_params);    
            void setPointcloud(const pcl::PointCloud<pcl::PointXYZI>& cloud); 
            void setPose(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);
            void setClosestPointOnLine(const Eigen::Vector3f& closest_pt);
            void setGoal(const Eigen::Vector3f& pose);
            void buildLookAheadTree();
    };
}
#endif