#ifndef A_START_TEST_H_
#define A_START_TEST_H_


#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <cstdio>
using namespace std;


static int angle_tolerance_astar = 15;

static int angle_steps_total = 360 / angle_tolerance_astar;
static int moves_possible = 4; // front , back and turn right 15 degrees, turn left 15 degrees

static int x_moves[moves_possible]={0,0,0,0};
static int y_moves[moves_possible]={1,-1,0,0};
static int angle_moves[moves_possible]={0,0,angle_tolerance_astar,-1*angle_tolerance_astar};


class node_a_star
{

	// current position
    int x_cord;
    int y_cord;
    int angle_cord;
    int G_val;
    int F_val;
    int angle_tolerance_astar; 

    public:
        node_a_star(int xpos, int ypos, int angle_pos, int g_start, int f_start) 
            {x_cord=xpos; y_cord=ypos; G_val=g_start; F_val=f_start; angle_cord=angle_pos; }

        virtual ~node_a_star();
    
        int getx() const {return x_cord;}
        int gety() const {return y_cord;}        
        int getangle() const {return angle_cord;}
        int getG() const {return G_val;}
        int getF() const {return F_val;}

        void updateF(const int target_x, const int target_y, const int target_angle_yaw)
        {
            F_val=G_val+get_h_val(target_x, target_y, target_angle_yaw)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextG(const int intended_dir) 
        {
            // TODO: here I have to change G value accorind to the movements forward, backward and rotation.. For now all are 10 
            G_val += 10;
        }
        
        // Estimation function for the remaining distance to the goal.
        int get_h_val(const int x_tgt, const int y_tgt, const int target_angle_yaw) const
        {
            static int xd, yd, d;
            static int ad;
            xd=xDest-xPos;
            yd=yDest-yPos;  


            static int distance_angle_opposite;

            if (target_angle_yaw >= angle_cord)
            {   // condition is if target angle is greater than starting
                distance_angle_opposite = (360 - target_angle_yaw) +  angle_cord;
                if( distance_angle_opposite < (target_angle_yaw - angle_cord) )
                {
                    ad = distance_angle_opposite / angle_tolerance_astar;
                } else {
                    // simple distance from goal to start  div by angle tolerance
                    ad = (target_angle_yaw -  angle_cord) / angle_tolerance_astar;
                }
            } else {
                distance_angle_opposite = (360 - angle_cord) +  target_angle_yaw;
                if( distance_angle_opposite < (angle_cord - target_angle_yaw) )
                {
                    ad = distance_angle_opposite / angle_tolerance_astar;
                } else {
                    // simple distance from start to goal div by angle tolerance
                    ad = (angle_cord -  target_angle_yaw) / angle_tolerance_astar;
                }
            }
            

            // Euclidian Distance
            //d=static_cast<int>(sqrt(xd*xd+yd*yd));

            //Manhattan distance + angular steps needed
            d=static_cast<int>( abs(xd)+abs(yd) + ad );
            return d;
        }


};
}



// Determine priority (in the priority queue)
bool operator<(const node_a_star &node_first, const node_a_star &node_second)
{
  return node_first.getF() > node_second.getF();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int StartPosX, const int StartPosY, const double StartPosYaw, const int GoalPosX, const int GoalPosY, const double GoalPosYaw )
{
    static priority_queue<node_a_star> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi=0; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x_nodes_astar, y_nodes_astar, angle_node_astar, angle_index_nodes_astar, x_next_child, y_next_child, angle_next_child;
    static int action_taken_step, action_backwards;
    static char c;


    static int StartPosYaw_discritized = ((int) StartPosYaw ) % 360;
    static int GoalPosYaw_discritized = ((int) GoalPosYaw) % 360;
    // Now rounding off to nearest step 
    StartPosYaw_discritized = ( ((StartPosYaw_discritized + angle_tolerance_astar/2) / angle_tolerance_astar) * angle_tolerance_astar ) % 360;
    GoalPosYaw_discritized = ( ((GoalPosYaw_discritized + angle_tolerance_astar/2) / angle_tolerance_astar) * angle_tolerance_astar ) % 360;


    // Here n = costmap_ros->getCostmap()->getSizeInCellsX() 
    // m = costmap_ros->getCostmap()->getSizeInCellsY() 
    static int a_star_closed_nodes[n][m][angle_steps_total]; 
    static int a_star_open_nodes[n][m][angle_steps_total]; 
    static int a_star_action_history[n][m][angle_steps_total]; 

    // reset the node maps
    for(y_nodes_astar=0;y_nodes_astar<m;y_nodes_astar++)
    {
        for(x_nodes_astar=0;x_nodes_astar<n;x_nodes_astar++)
        {
            for(angle_index_nodes_astar=0;angle_index_nodes_astar<angle_steps_total;angle_index_nodes_astar++)
            {
                a_star_closed_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar]=0;
                a_star_open_nodes[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar]=0;
            }
        }
    }



    // create the start node and push into list of open nodes
    n0start=new node_a_star(StartPosX, StartPosY, StartPosYaw_discritized, 0, 0);
    n0start->updateF(GoalPosX, GoalPosY,GoalPosYaw_discritized);
    pq[pqi].push(*n0start);
    open_nodes_map[x_nodes_astar][y_nodes_astar][angle_index_nodes_astar]=n0start->getF(); // mark it on the open nodes map

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
            string path="";
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

            if(!(x_next_child<0 || x_next_child>n-1 || y_next_child<0 || y_next_child>m-1 || robot_obstacle_not_hit 
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






#endif /* A_STAR_TEST_H_ */
