/*
 * MotionPrimitivesGlobalPlanner.h
 *
 *  Created on: Oct 28, 2016
 *      Author: andreas
 */

#ifndef SRC_MOTIONPRIMITIVESGLOBALPLANNER_H_
#define SRC_MOTIONPRIMITIVESGLOBALPLANNER_H_

#include <nav_core/base_global_planner.h>


#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <vector>




namespace motion_primitives_global_planner
{
/*
class cmp
{
	public:
	    bool operator()(const node_a_star& node_compare_one, const node_a_star& node_compare_two);

};
*/



class node_a_star
{

    public:
    	node_a_star();
        node_a_star(int xpos, int ypos, int angle_pos, int g_start, int f_start);
        virtual ~node_a_star();
    
        int getx() const;
        int gety() const;       
        int getangle() const;
        int getG() const;
        int getF() const;

        void updateF(const int target_x, const int target_y, const int target_angle_yaw);

        void nextG(const int intended_dir);
        
        // Estimation function for the remaining distance to the goal.
        int get_h_val(const int x_tgt, const int y_tgt, const int target_angle_yaw)  const;

        // for priority queue
        //friend bool operator< (const node_a_star& node_compare_one, const node_a_star& node_compare_two);
        // Determine priority (in the priority queue)
		bool operator<(const node_a_star& node_first_cmp) const;

    private:
	    // current position
	    int x_cord;
	    int y_cord;
	    int angle_cord;
	    // variables for A STAR
	    int G_val;
	    int F_val;
	    int angle_tolerance_node;

};



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
	std::vector<costmap_2d::MapLocation> give_robot_cells_from_map_coordinates(const int x_map_possible, const int y_map_possible, const int angle_map_possible );

	// A-star algorithm.
	// The route returned is a string of direction digits.
	std::string PathFindAStar( const int StartPosX, const int StartPosY, const double StartPosYaw, const int GoalPosX, const int GoalPosY, const double GoalPosYaw );

	// Determine priority (in the priority queue)
	/*
	class Compare
	{
		public:
		    bool operator<(const node_a_star& node_compare_one, const node_a_star& node_compare_two);
	};
	*/
	//friend bool operator< (const node_a_star& node_compare_one, const node_a_star& node_compare_two);
   

private:
	costmap_2d::Costmap2DROS* mycostmap_ros;


	// variables for A star below ... these will be initialized in initialization func 
	int angle_tolerance_astar;
	int angle_steps_total;
	int moves_possible;

	/*
	static int* x_moves;
	static int* y_moves;
	static int* angle_moves;
	*/




};
}








#endif /* SRC_MOTIONPRIMITIVESGLOBALPLANNER_H_ */
