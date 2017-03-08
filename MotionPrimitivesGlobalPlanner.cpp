/*
 * MotionPrimitivesGlobalPlanner.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: andreas
 */


#include <costmap_2d/testing_helper.h>

#include <tf/transform_datatypes.h>

#include <pluginlib/class_list_macros.h>
#include "motion_primitives_global_planner/MotionPrimitivesGlobalPlanner.h"
namespace motion_primitives_global_planner
{
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(motion_primitives_global_planner::MotionPrimitivesGlobalPlanner, nav_core::BaseGlobalPlanner)

MotionPrimitivesGlobalPlanner::MotionPrimitivesGlobalPlanner() {

}

MotionPrimitivesGlobalPlanner::~MotionPrimitivesGlobalPlanner() {
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<costmap_2d::MapLocation> MotionPrimitivesGlobalPlanner::give_robot_cells(const geometry_msgs::PoseStamped start)
{
	// This function outputs all the cells in the map that are currently occpied by the robot
	// It takes the current position of the robot's base 

	// For this I need to get the footprint of the robot's polygon transformed.... So the four vertices of the robot are transformed according to robot's position and orientation 
	// getRobotFootprintPolygon()  gives the foot print as the points of polygon if the robot is at 0,0 and no rotation .... in meters
	// I need to transform these points according to the orientation of robot via the function
	// transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,std::vector<geometry_msgs::Point>& oriented_footprint);

	//  and then convert them from world coordinates to map coordinate
	// finally I am going to get all the cells that are occupied by the robot
	// void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
	// I can also pad the robot .... at a later stage




	double yaw_rob = tf::getYaw(start.pose.orientation);

	/*
	std::cout << "\n here orientation =  \n" << start.pose.orientation;
	double yaw_degrees = yaw_rob * 180.0 / M_PI; // conversion to degrees
	if( yaw_degrees < 0 ) yaw_degrees += 360.0; // convert negative to positive angles
	std::cout << "\n   ... and yaw =   " << yaw_degrees << std::endl;
 	geometry_msgs::Quaternion q_from_yaw = tf::createQuaternionMsgFromYaw(yaw_rob);
	*/



	std::vector<geometry_msgs::Point> oriented_footprint;
	std::vector<geometry_msgs::Point> Robot_footprint_polygon = costmap_2d::toPointVector(mycostmap_ros->getRobotFootprintPolygon());

	costmap_2d::transformFootprint(start.pose.position.x, start.pose.position.y, yaw_rob, Robot_footprint_polygon, oriented_footprint);

	//std::cout << "\n\n Oriented Points are : " << costmap_2d::toPolygon(oriented_footprint) << "\n\n";
	std::vector<costmap_2d::MapLocation> polygon_in_map_cord;	// will send it to convexFillCells function

	for (int i_footprint_vertices = 0; i_footprint_vertices < oriented_footprint.size(); i_footprint_vertices++)
	{	costmap_2d::MapLocation temp_map_cord_polygon;	// temporary Maplocation data type to be pushed to a vector
		mycostmap_ros->getCostmap()->worldToMap(oriented_footprint[i_footprint_vertices].x, oriented_footprint[i_footprint_vertices].y, temp_map_cord_polygon.x, temp_map_cord_polygon.y);
		polygon_in_map_cord.push_back(temp_map_cord_polygon);
	}

	std::vector<costmap_2d::MapLocation> cells_occupied_by_robot;	// this should contain all the cells occupied by the robot
	mycostmap_ros->getCostmap()->convexFillCells(polygon_in_map_cord, cells_occupied_by_robot);

	
	/*
	// I can print all the cell coordinates of the robot as below
	std::cout << "\n\n Cells occpied by the robot are = ";

	for (int i_robot_cells = 0; i_robot_cells < cells_occupied_by_robot.size(); i_robot_cells++)
	{
		std::cout << "cell number : " << i_robot_cells <<  "   x_cord = " << cells_occupied_by_robot[i_robot_cells].x  << " y_cord = " << cells_occupied_by_robot[i_robot_cells].y << std::endl;
	}
	*/
	return cells_occupied_by_robot;

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<costmap_2d::MapLocation> MotionPrimitivesGlobalPlanner::give_robot_cells_from_map_coordinates(const int x_map_possible, const int y_map_possible, const int angle_map_possible )
{
	// This function outputs all the cells in the map that are currently occpied by the robot
	// It takes the current position of the robot as simple x, y, angle 

	// For this I need to get the footprint of the robot's polygon transformed.... So the four vertices of the robot are transformed according to robot's position and orientation 
	// getRobotFootprintPolygon()  gives the foot print as the points of polygon if the robot is at 0,0 and no rotation .... in meters
	// I need to transform these points according to the orientation of robot via the function
	// transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,std::vector<geometry_msgs::Point>& oriented_footprint);

	//  and then convert them from world coordinates to map coordinate
	// finally I am going to get all the cells that are occupied by the robot
	// void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
	// I can also pad the robot .... at a later stage



	std::vector<geometry_msgs::Point> oriented_footprint;
	std::vector<geometry_msgs::Point> Robot_footprint_polygon = costmap_2d::toPointVector(mycostmap_ros->getRobotFootprintPolygon());

 	double world_possible_x, world_possible_y;
 	mycostmap_ros->getCostmap()->mapToWorld(x_map_possible, y_map_possible, world_possible_x, world_possible_y);
	costmap_2d::transformFootprint(world_possible_x, world_possible_y, angle_map_possible, Robot_footprint_polygon, oriented_footprint);

	//std::cout << "\n\n Oriented Points are : " << costmap_2d::toPolygon(oriented_footprint) << "\n\n";
	std::vector<costmap_2d::MapLocation> polygon_in_map_cord;	// will send it to convexFillCells function

	for (int i_footprint_vertices = 0; i_footprint_vertices < oriented_footprint.size(); i_footprint_vertices++)
	{	costmap_2d::MapLocation temp_map_cord_polygon;	// temporary Maplocation data type to be pushed to a vector
		mycostmap_ros->getCostmap()->worldToMap(oriented_footprint[i_footprint_vertices].x, oriented_footprint[i_footprint_vertices].y, temp_map_cord_polygon.x, temp_map_cord_polygon.y);
		polygon_in_map_cord.push_back(temp_map_cord_polygon);
	}

	std::vector<costmap_2d::MapLocation> cells_occupied_by_robot;	// this should contain all the cells occupied by the robot
	mycostmap_ros->getCostmap()->convexFillCells(polygon_in_map_cord, cells_occupied_by_robot);

	
	/*
	// I can print all the cell coordinates of the robot as below
	std::cout << "\n\n Cells occpied by the robot are = ";

	for (int i_robot_cells = 0; i_robot_cells < cells_occupied_by_robot.size(); i_robot_cells++)
	{
		std::cout << "cell number : " << i_robot_cells <<  "   x_cord = " << cells_occupied_by_robot[i_robot_cells].x  << " y_cord = " << cells_occupied_by_robot[i_robot_cells].y << std::endl;
	}
	*/
	return cells_occupied_by_robot;

}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionPrimitivesGlobalPlanner::give_robot_cost(const std::vector<costmap_2d::MapLocation> robot_grid_cells, int &total_cost_current_position )
{
	//This function should give the cost of the robot's current position 
	// given all the cells it is occupying
	// If the robot's position collides with the wall or an obstacle, the cost of the robot should not be equal to 0
	total_cost_current_position = 0;
	int cell_cost_here = 0;
	bool obstacle_hit = false;
	for (int i_robot_cells = 0; i_robot_cells < robot_grid_cells.size(); i_robot_cells++)
	{	
		cell_cost_here = mycostmap_ros->getCostmap()->getCost(robot_grid_cells[i_robot_cells].x, robot_grid_cells[i_robot_cells].y);
		total_cost_current_position += cell_cost_here;
		
		// checking only if the cell is obstacle
		if (cell_cost_here == costmap_2d::LETHAL_OBSTACLE) 
		{	obstacle_hit=true;
	  	}
		//std::cout << "cell number : " << i_robot_cells <<  "  has value = " <<  cell_cost_here << "   x_cord = " << robot_grid_cells[i_robot_cells].x  << " y_cord = " << robot_grid_cells[i_robot_cells].y << std::endl;
	}

	return obstacle_hit;
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionPrimitivesGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& plan)
{
	ROS_INFO("makePlan");


	// here my own
	ROS_INFO("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT");

	// .......... below is the code to access some specific thingy
	double myxxx = start.pose.position.x;

	std::cout << "Starting poistion:\n";
	std::cout << start;
	std::cout << "Goal poistion:\n";
	std::cout << goal;


	unsigned int mymx, mymy;	// my map x and my map y coordinates

	/*
	// .............. to get robot location in cost map coordinates
	// mycostmap_ros->getCostmap()->worldToMap(start.pose.position.x, start.pose.position.y, mymx, mymy);
	*/


	mycostmap_ros->getCostmap()->worldToMap(start.pose.position.x, start.pose.position.y, mymx, mymy);
	std::cout << "\n\nIn Map coordinates, :\n  ----- Map x : " << mymx << "\n  ----- Map y : " << mymy << "\n\n";


	unsigned int goalmyx, goalmyy;
	mycostmap_ros->getCostmap()->worldToMap(goal.pose.position.x, goal.pose.position.y, goalmyx, goalmyy);
	std::cout << "\n\n Gola In Map coordinates, :\n  ----- Map x : " << goalmyx << "\n  ----- Map y : " << goalmyy << "\n\n";


	//geometry_msgs::Polygon getRobotFootprintPolygon()
	//std::cout << "\n\nPolygon Points are : " << mycostmap_ros->getRobotFootprintPolygon() << "\n\n";
		

	// here I am getting all the coordinates of the cells that are currently occupied by the robot
	std::vector<costmap_2d::MapLocation> mcells_occupied_by_robot = give_robot_cells(start);
	// Now the cost associated with robot's current position
	int cost_robot_pos = 0;
	bool wall_present = give_robot_cost(mcells_occupied_by_robot, cost_robot_pos );
	//std::cout << "\n\n\nHit the wall === " << wall_present << " :  and total cost = " << cost_robot_pos << std::endl;



	////// just testing
	std::cout << "\n Testing here orientation =  \n" << goal.pose.orientation;
	double yaw_rob_goal = tf::getYaw(goal.pose.orientation);
	double yaw_degrees = yaw_rob_goal * 180.0 / M_PI; // conversion to degrees
	if( yaw_degrees < 0 ) yaw_degrees += 360.0; // convert negative to positive angles
	std::cout << "\n   ... and yaw =   " << yaw_degrees << std::endl;
 	//geometry_msgs::Quaternion q_from_yaw = tf::createQuaternionMsgFromYaw(yaw_rob);
 	std::cout << "\n Testing ENDING ! \n";
 	///// testing testing














	 /*	---------------------	Size of Map  ----------------
	std::cout << " \n\n\nNow the size of the map .... cells x = " << mycostmap_ros->getCostmap()->getSizeInCellsX() << " : and the cells y = " 
		<< mycostmap_ros->getCostmap()->getSizeInCellsY() << " : size in meters X = " << mycostmap_ros->getCostmap()->getSizeInMetersX() << " : meters Y = " 
		<<	mycostmap_ros->getCostmap()->getSizeInMetersY() << std::endl;
	*/




	ROS_INFO("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU");

	// ......    getRobotPose takes input tf::Stamped<tf::Pose>
	// ......		but we have geometry_msgs::PoseStamped as start position, so below function converts them
	tf::Stamped<tf::Pose> start_pose_in_map;
	//tf::poseStampedMsgToTF(start, start_pose_in_map);
	// .......



	if(mycostmap_ros->getRobotPose(start_pose_in_map) ) {

		ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!   SOSOSOSOSO  SUCCESSFULL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		//double myxxx = start.pose.position.x;
		geometry_msgs::PoseStamped start_pose_in_map_msg_form;
 		tf::poseStampedTFToMsg(start_pose_in_map, start_pose_in_map_msg_form);
 		std::cout << start_pose_in_map_msg_form << std::endl;
 		ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!   ooooooooo  SUCCESSFULL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");


	} else{
		ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!NOT SUCCESSFULL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

	}



	// now returning a plan
	/////////////////  Just testing
	plan.push_back(start);
   	for (int i=0; i<20; i++){
     geometry_msgs::PoseStamped new_goal = goal;
     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

      new_goal.pose.position.x = -2.5+(0.05*i);
      new_goal.pose.position.y = -3.5+(0.05*i);

      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();

   	plan.push_back(new_goal);
   	}
   	plan.push_back(goal);
  	return true;

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionPrimitivesGlobalPlanner::initialize(std::string name,
			costmap_2d::Costmap2DROS* costmap_ros)
{
	ROS_INFO("initialize");

	// .......... just saving the pointer in the class so that I may be ableto use the cost_mapinother functions
	mycostmap_ros = costmap_ros;

	// here my own
	ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
	std::cout << name << std::endl;

	//Costmap2D* getCostmap()
	//costmap_2d::Costmap2DROS* justthismap = costmap_ros.getCostmap();
	double myreso = costmap_ros->getCostmap()->getResolution();
	std::cout << myreso << std::endl;

	ROS_INFO("iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii");


	//unsigned char* mymycostmap = costmap_ros->getCostmap()->getCharMap();
	//std::cout << mymycostmap << std::endl;
	
	unsigned char* aaamymycostmap = costmap_ros->getCostmap()->getCharMap();
	//printf("%s",aaamymycostmap);
	//std::cout << static_cast<const void*>(aaamymycostmap) << std::endl;
	std::cout << aaamymycostmap << std::endl;

	std::string ystr;
	ystr.append(reinterpret_cast<const char*>(aaamymycostmap));
	std::cout << ystr << std::endl;





	std::cout << "\n The size of cost map is  X # cells : " << costmap_ros->getCostmap()->getSizeInCellsX()	<< "    and is Y # cells : " << costmap_ros->getCostmap()->getSizeInCellsY() << std::endl;
	std::cout << "\n The size of cost map  in meters is  X # cells : " << costmap_ros->getCostmap()->getSizeInMetersX()	<< "    and is Y # cells : " << costmap_ros->getCostmap()->getSizeInMetersY() << std::endl;



	/*    
	// .......... below line is the line for printing the map
	printMap(*costmap_ros->getCostmap());
	*/

	//std::cout << std::isprint(mymycostmap) << std::endl;	
	ROS_INFO("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");


	//unsigned char* getCharMap();
  	//double getResolution();
  	//bool saveMap(std::string file_name);


  	// ..........................................
	//getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;



	// ...............Just for test...............
	/*
	initialized_ = false;
	if(!initialized_){
       costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
       costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_

      // initialize other planner parameters
       ros::NodeHandle private_nh("~/" + name);
       private_nh.param("step_size", step_size_, costmap_->getResolution());
       private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
       world_model_ = new base_local_planner::CostmapModel(*costmap_);

       initialized_ = true;
     }
     else
       ROS_WARN("This planner has already been initialized... doing nothing");
   */



}
}
