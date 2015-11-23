#include "vector"
#include "ros/ros.h"
#include <limits>

#include "std_msgs/String.h"

//#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "math.h"
#include "nav_msgs/OccupancyGrid.h"

using std::vector;

double startX;
double startY;

double goalX;
double goalY;

#define G_OFFSET_4  10

nav_msgs::OccupancyGrid *map;

struct Node
{
    int row;
    int col;

    int g; //past road cost
    int h;
    int f; //heuristic, F = G + H
    //struct Node* parent; //parent node, used for trace road
};


Node come_from[400][400];

void setCurrentPosition(const geometry_msgs::Pose::ConstPtr& msg)
{
    startX = msg.get()->position.x;
    startY = msg.get()->position.y;
}

void setGoalPosition(const geometry_msgs::Pose::ConstPtr& msg)
{
    goalX = msg.get()->position.x;
    goalY = msg.get()->position.y;
}

void setMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
//    int[] oneDiemensionalArrayMap = msg.get()->data;
//    int sizeOfRows = msg.get()->info.height;
//    int sizeOfCols = msg.get()->info.width;

      //std::cout<< "Setting the map" <<std::endl;
      map->data = msg.get()->data;
      map->info = msg.get()->info;
      map->header = msg.get()->header;
}


//if this nodeList has this value, then return "true"
bool itHas(vector<Node> nodeList, Node node)
{
    bool ithas = false;
    for(int i = 0; i < nodeList.size(); i++)
    {
        if(node.row == nodeList.at(i).row && node.col == nodeList.at(i).col)
        {
            ithas = true;
            break;
        }
    }
    return ithas;
}

class PathFinder
{
    Node start;
    Node goal;
    //nav_msgs::OccupancyGrid map;
    public:
    PathFinder(Node node1, Node node2)
    {
        start = node1;
        goal = node2;
        //ROS_INFO("PathFinder Intialized");
    }

    int hValue(Node current)
    {
        int h = (goal.col - current.col) + (goal.row - current.row);
        return h*10;
    }

    Node getBestChild(vector<Node> nodelist)
    {
        int bestValue = 999999;//std::numeric_limits<int>::max();
        int bestNum = 0;
        for(int i = 0; i < nodelist.size(); i++)
        {
            if(nodelist.at(i).f < bestValue)
            {
                bestValue = nodelist.at(i).f;
                bestNum = i;
            }
        }
        return nodelist.at(bestNum);
    }
    //if this point is passable, then return TRUE, if it is occupied, return false
    bool isCanMove(int row,int col)
    {
        if(map->data[row*map->info.width + col] > 10)
        {
            //System.err.println("Not available");
            return false;
        }
        else
        {
            //System.err.println("Available");
            return true;
        }
    }

    vector<Node> getNeighbours(Node current)
    {
        int row = current.row;
        int col = current.col;
        vector<Node> neighbours;
        //Up
        if(isCanMove(row+1,col))
        {
            Node newNode;
            newNode.row = row-1;
            newNode.col = col;
            newNode.g = 999999999;//std::numeric_limits<int>::max();
            //System.out.println("Up");
            neighbours.push_back(newNode);
        }
        //Down
        if(isCanMove(row-1,col))
        {
            Node newNode;
            newNode.row = row+1;
            newNode.col = col;
            newNode.g = 999999999;//std::numeric_limits<int>::max();
            //System.out.println("Down");
            neighbours.push_back(newNode);
        }
        //RightsetMap
        if(isCanMove(row,col+1))
        {
            Node newNode;
            newNode.row = row;
            newNode.col = col+1;
            newNode.g = 999999999;//std::numeric_limits<int>::max();
            //System.out.println("Right");
            neighbours.push_back(newNode);
        }
        //Left
        if(isCanMove(row,col-1))
        {
            Node newNode;
            newNode.row = row;
            newNode.col = col-1;
            newNode.g = 999999999;//std::numeric_limits<int>::max();
            //System.out.println("Left");
            neighbours.push_back(newNode);
        }
        return neighbours;
    }

    //This is used to determine if we go 8 directions or 4 directions
    int distance(Node current, Node child)
    {
        return 10 * abs(current.row - child.row) + abs(current.col - child.col);
    }

    int getPosition(vector<Node> nodeList, Node node)
    {
        int position;
        for(int i = 0; i < nodeList.size(); i++)
        {
            if(node.row = nodeList.at(i).row && node.col == nodeList.at(i).col)
            {
                position = i;
                break;
            }
        }
        return position;
    }

    //if the position of start or goal is invalid, then we return TRUE
    // ROW -> Height? COL -> Width?
    bool inValid(Node start, Node goal)
    {
        //std::cout << "Start Row & Col is : "<< start.row <<" & "<<start.col<<std::endl;
        //std::cout << "Goal Row & Col is : "<< goal.row <<" & "<<goal.col<<std::endl;
        //std::cout << "Map Height & Width is : "<< map->info.width <<" & "<<map->info.height<<std::endl;
        if(start.row >= map->info.height || start.col >= map->info.width ||
                goal.row >= map->info.height || goal.col >= map->info.width ||
                !isCanMove(start.row,start.col) || !isCanMove(goal.row,goal.col))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    nav_msgs::Path getPath()
    {
        //if this two point is invalid, then break
        if(inValid(start,goal))
        {
            ROS_INFO("Start or Goal is not valid");
            nav_msgs::Path nu;
            return nu;
        }
        //ROS_INFO("Begin to find the path");
        vector<Node> closeSet;
        vector<Node> openSet;
        closeSet.clear();
        openSet.clear();

        start.g = 0;
        start.h = hValue(start);
        start.f = start.g + start.h;
        come_from[start.row][start.col] = start;

        openSet.push_back(start);

        while(!openSet.empty())
        {
            //std::cout << "openSet size is "<< openSet.size()<<std::endl;
            Node current = getBestChild(openSet);
            //std::cout << "Current Point is: ( "<< current.row << " , "<<current.col <<" )" <<std::endl;
            //std::cout << "Goal Point is: ( "<< goal.row << " , "<<goal.col <<" )" <<std::endl;
            if(current.row == goal.row && current.col == goal.col)
            {
                ROS_INFO("Astar Destinaltion reached");
                return reconstruct(current);
            }
            //The following two lines are used to delete one certain element
            int position = getPosition(openSet,current);
            openSet.erase(openSet.begin() + position);

            closeSet.push_back(current);
            //std::cout << "CloseSet size is "<< closeSet.size()<<std::endl;

            vector<Node> neighbours = getNeighbours(current);

            for(int i = 0; i < neighbours.size(); i++)
            {
                //std::cout << "Neighbours size is "<< neighbours.size()   <<std::endl;
                //std::cout << "Neighbour Point is: ( "<< neighbours.at(i).row << " , "<<neighbours.at(i).col <<" )" <<std::endl;
                //ROS_INFO("Get neighbours?");
                Node child = neighbours.at(i);
               // std::cout << "Child is: ( "<< child.row << " , "<<child.col <<" )" <<std::endl;

                if(itHas(closeSet,child))
                {
                    //std::cout << "It has this child" <<std::endl;
                    continue;
                }


                int newG = current.g + G_OFFSET_4;  //distance(current,child);

                if (!itHas(openSet,child) || newG < child.g)
                {
                    //ROS_INFO("find the child");
                    //child.parent = &current;
                    child.g = newG;
                    child.f = child.g + hValue(child);
                    come_from[child.row][child.col] = current;

                    if (!itHas(openSet,child))
                    {
                        openSet.push_back(child);
                    }

                }

            }

        }
        nav_msgs::Path nu;
        return nu;
    }

    nav_msgs::Path reconstruct(Node current)
    {
        vector<Node> path;
        nav_msgs::Path finalPath;

        while(!(come_from[current.row][current.col].row == start.row && come_from[current.row][current.col].col == start.col))
        {
            //std::cout<< "Previous ROW is: "<< come_from[current.row][current.col].row << " and COL is: "<< come_from[current.row][current.col].col<<std::endl;
            Node parent = come_from[current.row][current.col];
            path.push_back(parent);
            current = come_from[current.row][current.col];
        }


        // Should this be just larger than 0 or it could equal 0
        for(int i = path.size() - 1; i > 0; i--)
        {
           geometry_msgs::PoseStamped p;
           p.pose.position.x = path.at(i).row;
           p.pose.position.y = path.at(i).col;
           p.pose.orientation.x = 0;
           p.pose.orientation.y = 0;
           p.pose.orientation.z = 0;
           p.pose.orientation.w = 1;

           finalPath.poses.push_back(p);
        }

        finalPath.header.stamp = ros::Time::now();
        //Frame_ID maybe wrong, as I could not show this in our case
        finalPath.header.frame_id = "/map";

        return finalPath;
    }
};

nav_msgs::Path simpilifyPath(nav_msgs::Path path)
{
    nav_msgs::Path newPath;

    int pathSize = path.poses.size();

    for(int i = 0; i < pathSize - 1; i++)
    {
        for(int j = 1; j < pathSize; j++)
        {
            if(path.poses.at(i).pose.position.y != path.poses.at(j).pose.position.y )
            {
                //ROS_INFO("X value changes");
                geometry_msgs::PoseStamped p;
                p.pose.position.x = path.poses.at(i).pose.position.x;
                p.pose.position.y = path.poses.at(i).pose.position.y;
                p.pose.orientation.x = 0;
                p.pose.orientation.y = 0;
                p.pose.orientation.z = 0;
                p.pose.orientation.w = 1;

                newPath.poses.push_back(p);
            }
        }
    }

    newPath.header.stamp = ros::Time::now();
    newPath.header.frame_id = "/map";

    return newPath;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Astar");
    ros::NodeHandle handle;

    // function for calculate the Current Position: baselink (0.0) to the map frame (TF)

    //ros::Subscriber path_sub_current = handle.subscribe<geometry_msgs::Pose>("/currentPosition",1000,setCurrentPosition);
    //ros::Subscriber path_sub_goal = handle.subscribe<geometry_msgs::Pose>("/goalPosition",1000,setGoalPosition);

    //Set map;
    map = new nav_msgs::OccupancyGrid();
    map->info.height = 0;
    map->info.width = 0;
    ros::Subscriber path_sub_map = handle.subscribe<nav_msgs::OccupancyGrid>("/map",5,setMap);
    ros::Publisher path_pub = handle.advertise<nav_msgs::Path>("/Astar/path", 10);
    
    Node start;
    Node goal;

//    start.row = startX;
//    start.col = startY;
//    goal.row = goalX;
//    goal.col = goalY;

    //row stands for the y-coordinate; col stands for the x value;
    start.row = 20;
    start.col = 20;
    //row:60, col:200 is the one we couldn't find the path
    goal.row = 20;//40;
    goal.col = 220;//20;

    PathFinder pf(start,goal);

    ros::Rate loop_rate(1);

    nav_msgs::Path path;
     nav_msgs::Path simpilifiedPath;
	while (ros::ok())
	{
        if(map->data.size() < 1000)
        {
            ROS_INFO("Map is not yet intialized");
        }
        else
        {
          ROS_INFO("Planning a path...");
          path =pf.getPath();
          if(path.poses.size() > 0)
          {
             simpilifiedPath = simpilifyPath(path);
          }
          path_pub.publish(simpilifiedPath);
          //path_pub.publish(path);
        }
        ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
