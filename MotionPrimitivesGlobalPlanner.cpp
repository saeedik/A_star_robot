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




    node_a_star::node_a_star(int xpos, int ypos, int angle_pos, int g_start, int f_start) 
    	{x_cord=xpos; y_cord=ypos; G_val=g_start; F_val=f_start; angle_cord=angle_pos; angle_tolerance_node = 15; }

    node_a_star::~node_a_star() { }
    
    int node_a_star::getx()  const {return x_cord;}
    int node_a_star::gety()  const {return y_cord;}        
    int node_a_star::getangle()  const {return angle_cord;}
    int node_a_star::getG()  const  {return G_val;}
    int node_a_star::getF()  const {return F_val;}

    void node_a_star::updateF(const int target_x, const int target_y, const int target_angle_yaw)
    {	F_val=G_val+get_h_val(target_x, target_y, target_angle_yaw)*10; //A*
    }

    // give better priority to going strait instead of diagonally
    void node_a_star::nextG(const int intended_dir) 
    {
        // TODO: here I have to change G value accorind to the movements forward, backward and rotation.. For now all are 10 
        G_val += 10;
    }
        
    // Estimation function for the remaining distance to the goal.
    int node_a_star::get_h_val(const int x_tgt, const int y_tgt, const int target_angle_yaw)  const
    {
        static int xd, yd, d;
        static int ad;
        xd=x_tgt-x_cord;
        yd=y_tgt-y_cord;  

		static int distance_angle_opposite;

        if (target_angle_yaw >= angle_cord)
        {   // condition is if target angle is greater than starting
            distance_angle_opposite = (360 - target_angle_yaw) +  angle_cord;
            if( distance_angle_opposite < (target_angle_yaw - angle_cord) )
            {
            	ad = distance_angle_opposite / angle_tolerance_node;
            } else {
                // simple distance from goal to start  div by angle tolerance
                ad = (target_angle_yaw -  angle_cord) / angle_tolerance_node;
            }
        } else {
            distance_angle_opposite = (360 - angle_cord) +  target_angle_yaw;
            if( distance_angle_opposite < (angle_cord - target_angle_yaw) )
            {
                ad = distance_angle_opposite / angle_tolerance_node;
            } else {
                // simple distance from start to goal div by angle tolerance
                ad = (angle_cord -  target_angle_yaw) / angle_tolerance_node;
            }
        }
            
            // Euclidian Distance
            //d=static_cast<int>(sqrt(xd*xd+yd*yd));

            //Manhattan distance + angular steps needed
            d=static_cast<int>( abs(xd)+abs(yd) + ad );
            return d;
    }

    /*
    bool node_a_star::operator< (const node_a_star& node_compare_one, const node_a_star& node_compare_two)
    {
    	return node_compare_one.getF() > node_compare_two.getF();
    }
	*/
	bool node_a_star::operator<(const node_a_star& node_first_cmp) const
	{
		return F_val > node_first_cmp.getF();
	}


namespace motion_primitives_global_planner
{
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(motion_primitives_global_planner::MotionPrimitivesGlobalPlanner, nav_core::BaseGlobalPlanner)

MotionPrimitivesGlobalPlanner::MotionPrimitivesGlobalPlanner() {

}

MotionPrimitivesGlobalPlanner::~MotionPrimitivesGlobalPlanner() {

	/*
	if(x_moves) // True if x_moves is not a null pointer
    	delete[] x_moves; 
    if(y_moves) // True if x_moves is not a null pointer
    	delete[] y_moves; 
    if(angle_moves) // True if x_moves is not a null pointer
    	delete[] angle_moves; 
	*/

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


	// variables for A star below
	angle_tolerance_astar = 15;
	angle_steps_total = 360 / angle_tolerance_astar;
	moves_possible = 4; // front , back and turn right 15 degrees, turn left 15 degrees

	/*
	x_moves = new int[4] {0,0,0,0};
	y_moves = new int[4] {1,-1,0,0};
	angle_moves = new int[4] {0,0,angle_tolerance_astar,-1*angle_tolerance_astar};
	*/
	// till here A star variables


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


/*
bool MotionPrimitivesGlobalPlanner::Compare::operator<(const node_a_star& node_compare_one, const node_a_star& node_compare_two)
{
	return node_compare_one.getF() > node_compare_two.getF();
}
*/
/*
bool cmp::operator()(const node_a_star& node_compare_one, const node_a_star& node_compare_two)
{
	return node_compare_one.getF() < node_compare_two.getF();
}
*/


// below is the definition of the function for A star algo
std::string MotionPrimitivesGlobalPlanner::PathFindAStar( const int StartPosX, const int StartPosY, const double StartPosYaw, const int GoalPosX, const int GoalPosY, const double GoalPosYaw )
{

	//static std::priority_queue<node_a_star, std::vector<node_a_star>, cmp> pq[2]; // list of open (not-yet-tried) nodes
    static std::priority_queue<node_a_star> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi=0; // pq index
    static node_a_star* n0;
    static node_a_star* n0start;
    static node_a_star* m0;
    static int i, j, x_nodes_astar, y_nodes_astar, angle_node_astar, angle_index_nodes_astar, x_next_child, y_next_child, angle_next_child;
    static int action_taken_step, action_backwards;
    static char c;



    const int mymap_x_size = mycostmap_ros->getCostmap()->getSizeInCellsX();

    const int mymap_y_size = mycostmap_ros->getCostmap()->getSizeInCellsY();
    const int node_size_angle_steps_total = angle_steps_total;


    static int StartPosYaw_discritized = ((int) StartPosYaw ) % 360;
    static int GoalPosYaw_discritized = ((int) GoalPosYaw) % 360;
    // Now rounding off to nearest step 
    StartPosYaw_discritized = ( ((StartPosYaw_discritized + angle_tolerance_astar/2) / angle_tolerance_astar) * angle_tolerance_astar ) % 360;
    GoalPosYaw_discritized = ( ((GoalPosYaw_discritized + angle_tolerance_astar/2) / angle_tolerance_astar) * angle_tolerance_astar ) % 360;


    // Here n = costmap_ros->getCostmap()->getSizeInCellsX() 
    // m = costmap_ros->getCostmap()->getSizeInCellsY() 
    /*
    int * a_star_closed_nodes = new int [mymap_x_size][mymap_y_size][node_size_angle_steps_total]; 
    int * a_star_open_nodes = new int [mymap_x_size][mymap_y_size][node_size_angle_steps_total]; 
    int * a_star_action_history = new int [mymap_x_size][mymap_y_size][node_size_angle_steps_total]; 

    // reset the node maps
    for(y_nodes_astar=0;y_nodes_astar<mymap_y_size;y_nodes_astar++)
    {
        for(x_nodes_astar=0;x_nodes_astar<mymap_x_size;x_nodes_astar++)
        {
            for(angle_index_nodes_astar=0;angle_index_nodes_astar<angle_steps_total;angle_index_nodes_astar++)
            {
                a_star_closed_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar]=0;
                a_star_open_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar]=0;
            }
        }
    }
	*/

	std::vector<std::vector<std::vector<int> > > a_star_closed_nodes(mymap_x_size, std::vector<std::vector<int> > (mymap_y_size, std::vector<int>(angle_steps_total,0)));
	std::vector<std::vector<std::vector<int> > > a_star_open_nodes(mymap_x_size, std::vector<std::vector<int> > (mymap_y_size, std::vector<int>(angle_steps_total,0)));
	std::vector<std::vector<std::vector<int> > > a_star_action_history(mymap_x_size, std::vector<std::vector<int> > (mymap_y_size, std::vector<int>(angle_steps_total,0)));


    	static int x_moves[] = {0,0,0,0};
	static int y_moves[] = {1,-1,0,0};
	static int angle_moves[] = {0,0,angle_tolerance_astar,-1*angle_tolerance_astar};



    // create the start node and push into list of open nodes
    n0start=new node_a_star(StartPosX, StartPosY, StartPosYaw_discritized, 0, 0);
    n0start->updateF(GoalPosX, GoalPosY,GoalPosYaw_discritized);
    pq[pqi].push(*n0start);
    a_star_open_nodes[mymap_x_size-1][mymap_y_size-1][angle_steps_total-1]=n0start->getF(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes

        n0=new node_a_star( pq[pqi].top().getx(), pq[pqi].top().gety(), pq[pqi].top().getangle(), pq[pqi].top().getG(), pq[pqi].top().getF());

        x_nodes_astar=n0->getx(); y_nodes_astar=n0->gety(); angle_node_astar=n0->getangle(); angle_index_nodes_astar= n0->getangle() / angle_tolerance_astar;

        pq[pqi].pop(); // remove the node from the open list
        a_star_open_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar]=0;
        // mark it on the closed nodes map
        a_star_closed_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar]=1;

        // quit searching when the goal state is reached
        // if((*n0).estimate(xFinish, yFinish) == 0)
        if( x_nodes_astar==GoalPosX && y_nodes_astar==GoalPosY && angle_node_astar==GoalPosYaw_discritized ) 
        {
            std::cout << " GoalPosX and GoalPosY reached \n";

            // generate the path from finish to start
            // by following the directions
            std::string path="";
            while(!(x_nodes_astar==StartPosX && y_nodes_astar==StartPosY && angle_node_astar==StartPosYaw_discritized ))
            {
                // 
                // the action that was taken to reach this node
                action_taken_step = a_star_action_history[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar / angle_tolerance_astar];
                if (action_taken_step == 0)
                {   // robot would have moved upwards ... the parent node is then one downwards
                    c = '0'+1;
                    action_backwards = 1;
                } else if (action_taken_step == 1)
                {   // robot would have moved downwards ... the parent node is then one upwards
                    c = '0'+0;
                    action_backwards = 0;
                } else if (action_taken_step == 2)
                {   // robot would have rotated right ... the parent node is then one rotation left
                    c = '0'+3;
                    action_backwards = 3;
                } else if (action_taken_step == 3)
                {   // robot would have rotated right ... the parent node is then one rotation left
                    c = '0'+2;
                    action_backwards = 2;
                } 

                path=c+path;
                x_nodes_astar += x_moves[action_backwards];
                y_nodes_astar += y_moves[action_backwards];
                angle_node_astar = ( angle_node_astar + angle_moves[i] ) % 360;
                if(angle_node_astar < 0)
                {   angle_node_astar = (angle_node_astar + 360) % 360;  }
                // 
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();           
            return path;
            
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<moves_possible;i++)
        {

            x_next_child = x_nodes_astar + x_moves[i];
            y_next_child = y_nodes_astar + y_moves[i];
            angle_next_child = ( angle_node_astar + angle_moves[i] ) % 360;
            if(angle_next_child < 0)
            {   angle_next_child = (angle_next_child + 360) % 360;  }

            // TODO: here send the x and y coordinates of the possible robot position to the file MotionPrimitiesGlobalPlanner.cpp 
            // use MotionPrimitivesGlobalPlanner::give_robot_cells_from_map_coordinates(x_next_child,y_next_child,angle_next_child) to get cells occupied by the robot and then 
            // robot_obstacle_not_hit = MotionPrimitivesGlobalPlanner::give_robot_cost(const std::vector<costmap_2d::MapLocation> robot_grid_cells, int &total_cost_current_position )
            static bool robot_obstacle_not_hit = true;

            std::vector<costmap_2d::MapLocation> mcells_occupied_by_robot_next_child = give_robot_cells_from_map_coordinates(x_next_child, y_next_child, angle_next_child );
            int cost_next_move_total;
            robot_obstacle_not_hit = give_robot_cost(mcells_occupied_by_robot_next_child, cost_next_move_total );



            if(!(x_next_child<0 || x_next_child>mymap_x_size-1 || y_next_child<0 || y_next_child>mymap_y_size-1 || robot_obstacle_not_hit 
                || a_star_closed_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar  / angle_tolerance_astar]==1) )
            {
                // generate a child node
                m0=new node_a_star( x_next_child, y_next_child, angle_next_child, n0->getG(), n0->getF());

                m0->nextG(i);
                m0->updateF(GoalPosX, GoalPosY, GoalPosYaw_discritized);

                // if it is not in the open list then add into that
                if( a_star_open_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar / angle_tolerance_astar]==0)
                {
                    a_star_open_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar / angle_tolerance_astar]=m0->getF();
                    pq[pqi].push(*m0);
                    // save the action taken at this step 
                    a_star_action_history[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar / angle_tolerance_astar]=i;
                }
                else if(a_star_open_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar / angle_tolerance_astar] > m0->getF())
                {
                    // update the priority info
                    a_star_open_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar / angle_tolerance_astar] = m0->getF();
                    // save the action taken at this step 
                    a_star_action_history[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar / angle_tolerance_astar]=i;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead


                    while(!(pq[pqi].top().getx()==x_nodes_astar && pq[pqi].top().gety()==y_nodes_astar && 
                        pq[pqi].top().getangle()==angle_index_nodes_astar ))
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pq[pqi].pop(); // remove the wanted node
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    delete n0start;




    return ""; // no route found
}

}
