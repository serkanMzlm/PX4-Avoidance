#ifndef __LOCAL_PLANNER_TYPE_H__
#define __LOCAL_PLANNER_TYPE_H__

#include <Eigen/Dense>
#include <vector>

#include <rclcpp/time.hpp>

#include "common.h"

namespace avoidance
{

	enum waypoint_choice
	{
		hover,
		tryPath,
		direct,
		reachHeight
	};

	struct avoidanceOutput
	{
		float cruise_velocity;		 // mission cruise velocity
		rclcpp::Time last_path_time; // finish built time for the VFH+* tree

		std::vector<Eigen::Vector3f> path_node_positions; // array of tree nodes  position, each node is the
														  // minimum cost node for each tree depth level
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

	struct costParameters
	{
		float yaw_cost_param = 0.5f;
		float pitch_cost_param = 3.f;
		float velocity_cost_param = 1.5f;
		float obstacle_cost_param = 5.0f;
	};
}
#endif