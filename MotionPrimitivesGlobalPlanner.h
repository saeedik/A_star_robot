/*
 * MotionPrimitivesGlobalPlanner.h
 *
 *  Created on: Oct 28, 2016
 *      Author: andreas
 */

#ifndef SRC_MOTIONPRIMITIVESGLOBALPLANNER_H_
#define SRC_MOTIONPRIMITIVESGLOBALPLANNER_H_

#include <nav_core/base_global_planner.h>

namespace motion_primitives_global_planner
{

class MotionPrimitivesGlobalPlanner: public nav_core::BaseGlobalPlanner
{
public:
	MotionPrimitivesGlobalPlanner();
	virtual ~MotionPrimitivesGlobalPlanner();

	/**
	 * @brief Given a goal pose in the world, compute a plan
	 * @param start The start pose
	 * @param goal The goal pose
	 * @param plan The plan... filled by the planner
	 * @return True if a valid plan was found, false otherwise
	 */
	virtual bool makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& plan);

	/**
	 * @brief  Initialization function for the BaseGlobalPlanner
	 * @param  name The name of this planner
	 * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
	 */
	virtual void initialize(std::string name,
			costmap_2d::Costmap2DROS* costmap_ros);



	std::vector<costmap_2d::MapLocation> give_robot_cells(const geometry_msgs::PoseStamped start);
	bool give_robot_cost(const std::vector<costmap_2d::MapLocation> robot_grid_cells, int &total_cost_current_position );




private:
	costmap_2d::Costmap2DROS* mycostmap_ros;

	const int theta_tolerance = 15; 	// if robot is 15 degrees near to goal, then it is acceptable
	const double cost_simle_movement = 1.0;
	const double cost_angular_movement = 1.0;


};
}
#endif /* SRC_MOTIONPRIMITIVESGLOBALPLANNER_H_ */
