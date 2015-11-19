#include "vector"
#include "ros/ros.h"
#include <limits>

#include "std_msgs/String.h"

#include <tf/transform_listener.h>
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
#define G_OFFSET_8  14

nav_msgs::OccupancyGrid *map;

struct Node
{
    int row;
    int col;

    int g; //past road cost
    int h;
    int f; //heuristic, F = G + H
    struct Node* parent; //parent node, used for trace road
};

void setCurrentPosition(const geometry_msgs::Pose::ConstPtr& msg)
{
    // have to check the format of the map. Should I use double or int?
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

      std::cout<< "Setting the map" <<std::endl;
      ROS_INFO("Setting");
      map->data = msg.get()->data;
      map->info = msg.get()->info;
      map->header = msg.get()->header;
}


//if this nodeList has this value, then return "true"
bool itHas(vector<Node> nodeList, Node node)
{
    bool itHas = false;
    for(int i = 0; i < nodeList.size(); i++)
    {
        if(node.row = nodeList.at(i).row && node.col == nodeList.at(i).col)
        {
            itHas = true;
            break;
        }
    }
    return itHas;
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
        if(map->data[col*map->info.width + row] > 10)
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
    bool inValid(Node start, Node goal)
    {
        if(start.row >= map->info.width && start.col >= map->info.height &&
                goal.row >= map->info.width && goal.col >= map->info.height &&
                isCanMove(start.row,start.col) && isCanMove(goal.row,goal.col))
            return true;
        else
            return false;
    }

    nav_msgs::Path getPath()
    {
        //if this two point is invalid, then break
        if(inValid(start,goal))
        {
            nav_msgs::Path nu;
            return nu;
        }

        vector<Node> closeSet;
        vector<Node> openSet;
        closeSet.clear();
        openSet.clear();

        start.g = 0;
        start.h = hValue(start);
        start.f = start.g + start.h;
        openSet.push_back(start);

        while(openSet.size() > 0)
        {
            Node current = getBestChild(openSet);
            if(current.row == goal.row && current.col == goal.col)
            {
                ROS_INFO("Astar Destinaltion reached");
                return reconstruct(current);
            }
            //The following two lines are used to delete one certain element
            int position = getPosition(openSet,current);
            openSet.erase(openSet.begin() + position);

            closeSet.push_back(current);

            vector<Node> neighbours = getNeighbours(current);

            for(int i = 0; i < neighbours.size(); i++)
            {
                Node child = neighbours.at(i);
                /*
                if(closeSet.contains(child))
                    continue;
                 */

                if(itHas(closeSet,child))
                    continue;

                int newG = current.g + G_OFFSET_4;  //distance(current,child);
                //TODO- Check this part
                if (!itHas(openSet,child) || newG < child.g)
                {
                    child.parent = &current;
                    child.g = newG;
                    child.f = child.g + hValue(child);

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
        while(current.parent != NULL)
        {
            path.push_back(*current.parent);
            //TODO
            current = *current.parent;
        }
        // Should this be just larger than 0 or it could equal 0
        for(int i = path.size(); i > 0; i--)
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
        finalPath.header.frame_id = "map";

        return finalPath;
    }
};


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

    start.row = 10;
    start.col = 12;
    goal.row = 100;
    goal.col = 100;
    //PathFinder pathfind = new PathFinder(start,goal);
    PathFinder pf(start,goal);
    ROS_INFO("Here");
    //std::cout<< "Start X: " << start.row <<std::endl;

    nav_msgs::Path path =pf.getPath();

    ros::Rate loop_rate(10);


	while (ros::ok())
	{
        if(path.poses.size() < 0)
        {
            ROS_INFO("No Path");
        }
        else
        {
          path_pub.publish(path);
        }

		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}

	return 0;
}
