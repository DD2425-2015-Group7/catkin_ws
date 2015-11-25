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
#include "map_tools/GetMap.h"
#include "path_planning/GetPath.h"
#include <tf/transform_listener.h>
#include "tf/tf.h"

ros::ServiceClient *map_client;

using std::vector;

double startX;
double startY;

double goalX;
double goalY;

double cell_size;

double pathDistance = 0;

#define G_OFFSET_4  1

nav_msgs::OccupancyGrid *map;
geometry_msgs::PoseStamped startPose;
tf::TransformListener *tf_listener;

bool updateMap(void)
{
    map_tools::GetMap srv2;
    srv2.request.type.data = "distance";
    if (map_client->call(srv2)){
        *map = srv2.response.map;
        cell_size = map->info.resolution;
    }else{
        ROS_ERROR("Failed to call service GetMap (distance_obj).");
        return false;
    }
    return true;
}


struct Node
{
    int row;
    int col;

    int g; //past road cost
    int h;

    int o; // O stands for the distance between the current point and the nearest obstacle

    int f; //heuristic, f = g + h + o
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
      //std::cout<< "Setting the map" <<std::endl;
      map->data = msg.get()->data;
      map->info = msg.get()->info;
      map->header = msg.get()->header;

      cell_size = msg.get()->info.resolution;
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

        return h;
    }

    int oValue(Node current)
    {
        // As for the toal f value, the smaller, the better, while the obstacle of distance should be the larger the better, so I reverse this value
        int o = (50 - (map->data[current.row*map->info.width + current.col]))/5;
        return o;
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
        if(row < 0)
        {
            return false;
        }
        if(row >= map->info.height)
        {
            return false;
        }
        if(col < 0)
        {
            //ROS_INFO("col < 0");
            return false;
        }
        if(col >= map->info.width)
        {
            return false;
        }
        if(map->data[row*map->info.width + col] <= 15)
        {
            return false;
        }
        else
        {
            return true;
        }


        /*
        assert(row >= 0);
        assert(col >= 0);
        assert(col < map->info.width);
        assert(row < map->info.height);

        if(map->data[row*map->info.width + col] > 15)
        {
            //System.err.println("Not available");
            return true;
        }
        else
        {
            //System.err.println("Available");
            return false;
        }
        */
    }

    vector<Node> getNeighbours(Node current)
    {
        int row = current.row;
        int col = current.col;

        vector<Node> neighbours;
        //Up
        if(isCanMove(row+1,col))
        {
            //std::cout<<"Up"<<std::endl;
            Node newNode;
            newNode.row = row+1;
            newNode.col = col;
            newNode.g = 999999999;//std::numeric_limits<int>::max();
            //std::cout<<"Up"<<std::endl;
            neighbours.push_back(newNode);
        }
        //Down
        if(isCanMove(row-1,col))
        {
            Node newNode;
            newNode.row = row-1;
            newNode.col = col;
            newNode.g = 999999999;//std::numeric_limits<int>::max();
            //System.out.println("Down");
            neighbours.push_back(newNode);
        }
        //Right
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
        start.o = oValue(start);
        start.f = start.g + start.h + start.o;
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


                int newG = current.g + G_OFFSET_4;

                if (!itHas(openSet,child))
                {
                    openSet.push_back(child);
                }
                else if(newG >= child.g)
                {
                    continue;
                }

                come_from[child.row][child.col] = current;
                child.g = newG;
                child.f = child.g + hValue(child) + oValue(child);
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
            //std::cout<< "Original Path No." << i <<" point is X: "<< path.at(i).col << " Y :" << path.at(i).row<< std::endl;
           geometry_msgs::PoseStamped p;
           //path.x should be the col value, right? TODO: check how the pub_points works. The coordinate must stay the same
           p.pose.position.x = path.at(i).col * cell_size;
           p.pose.position.y = path.at(i).row * cell_size;
           p.pose.orientation.x = 0;
           p.pose.orientation.y = 0;
           p.pose.orientation.z = 0;
           p.pose.orientation.w = 1;

           finalPath.poses.push_back(p);
        }

        //Add the goal point in the path. For some reason, the path will miss the two points(one is the goal and one the one before the goal), this could be a TODO;
        geometry_msgs::PoseStamped p;
        p.pose.position.x = goal.col * cell_size;
        p.pose.position.y = goal.row * cell_size;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;

        finalPath.poses.push_back(p);

        finalPath.header.stamp = ros::Time::now();
        finalPath.header.frame_id = "/map";
        std::cout<<"Original Path Size is: "<< finalPath.poses.size()<<std::endl;
        //Show the Original path
        /*
        for(int i = 0; i < finalPath.poses.size(); i++)
        {
            //std::cout<< "Original Path No." << i <<" point is X: "<< finalPath.poses.at(i).pose.position.x << " Y :" << finalPath.poses.at(i).pose.position.y << std::endl;
        }
        */

        return finalPath;
    }
};

nav_msgs::Path simpilifyPath(nav_msgs::Path path)
{
    nav_msgs::Path newPath;

    //This loop is used to store the "corner" point(either x or y value changes)
    //checkY means we will check the y difference for the two points
    bool checkY = true;
    for(int i = 0; i < path.poses.size() - 1; i++)
    {
       int j = i + 1;
       if(checkY)
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
               checkY = false;
            }
       }
       else
       {
           if(path.poses.at(i).pose.position.x != path.poses.at(j).pose.position.x )
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
               checkY = true;
            }
       }
    }

    //Show the new path(corner Point)
    std::cout<<"Simple Path Size is: "<< newPath.poses.size()<<std::endl;
    for(int i = 0; i < newPath.poses.size(); i++)
    {
        //std::cout<< "Corner Path No." << i <<" point is X: "<< newPath.poses.at(i).pose.position.x << " Y :" << newPath.poses.at(i).pose.position.y << std::endl;
    }


    nav_msgs::Path finalPath;
    //Simplify the path further (ignore the point that is too close)
    int start = 0;
    int end = start + 1;
    //Add the firstPoint (start Point) to the path
    geometry_msgs::PoseStamped p;
    p.pose.position.x = newPath.poses.at(0).pose.position.x;
    p.pose.position.y = newPath.poses.at(0).pose.position.y;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;

    finalPath.poses.push_back(p);

    while(end < newPath.poses.size())
    {
        //std::cout<< "End Value : "<< end <<std::endl;
        //std::cout<< "Start Value : "<< start <<std::endl;
        double distanceBetweenPoints = sqrt((newPath.poses.at(start).pose.position.y - newPath.poses.at(end).pose.position.y)*(newPath.poses.at(start).pose.position.y - newPath.poses.at(end).pose.position.y) +
                (newPath.poses.at(start).pose.position.x - newPath.poses.at(end).pose.position.x)*(newPath.poses.at(start).pose.position.x - newPath.poses.at(end).pose.position.x));
        //std::cout<< "Distance : "<< distanceBetweenPoints <<std::endl;

        //If the distance between those two points are less than 0.1 meters
        if(distanceBetweenPoints < 0.1)
        {
            end++;
        }
        else
        {
            //std::cout<< "The point we add : "<< "( " << newPath.poses.at(end).pose.position.x<< " , "<<newPath.poses.at(end).pose.position.y <<" )"<<std::endl;
            //std::cout<< "End Value : "<< end <<std::endl;
            //std::cout<< "Path size is : "<< newPath.poses.size() << std::endl;
            p.pose.position.x = newPath.poses.at(end).pose.position.x;
            p.pose.position.y = newPath.poses.at(end).pose.position.y;
            p.pose.orientation.x = 0;
            p.pose.orientation.y = 0;
            p.pose.orientation.z = 0;
            p.pose.orientation.w = 1;

            finalPath.poses.push_back(p);

            start = end;
            end = start + 1;
        }
    }
    // TODO - There is some risk that the destination will be removed. If this happens, how to solve this
    // 1. Remove the last point and add the destination point; <-- I personally prefer this one
    // 2. Just add the destination Point.
    // And here we could use the "newPath last point" as the goal value
    if(finalPath.poses.back().pose.position.x != newPath.poses.back().pose.position.x &&
           finalPath.poses.back().pose.position.y != newPath.poses.back().pose.position.y )
    {
        finalPath.poses.pop_back(); // Delete the last point
        finalPath.poses.push_back(newPath.poses.back()); // Add the last point of the newPath
    }


    std::cout<< "FinalPath size is : "<< finalPath.poses.size() << std::endl;
//    for(int i = 0; i < finalPath.poses.size(); i++)
//    {
//        std::cout<< "Final Path No." << i <<" point is X: "<< finalPath.poses.at(i).pose.position.x << " Y :" << finalPath.poses.at(i).pose.position.y << std::endl;
//    }
    //This is for the final Path
    finalPath.header.stamp = ros::Time::now();
    finalPath.header.frame_id = "/map";
    return finalPath;


    /*
    //newPath could be removed, I think
    newPath.header.stamp = ros::Time::now();
    newPath.header.frame_id = "/map";
    return newPath;
    */

}

nav_msgs::Path servicePath(geometry_msgs::Pose &msg)
{
    //The Pose msg is in meters, so multiply this by 100
    int goalCol = msg.position.y / cell_size;
    int goalRow = msg.position.x / cell_size;
    //TODO - TF function to get the start point;
    Node start;
//    start.row = 20;
//    start.col = 20;
    std::string TargetFrameName = "/map";
    std::string CurrentFrame = "/base_link";

    geometry_msgs::PoseStamped p;
    p.header.frame_id = CurrentFrame;
    p.header.stamp = ros::Time(0);
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 1.0;

    try
    {
        tf_listener->waitForTransform(TargetFrameName, CurrentFrame, ros::Time(0), ros::Duration(1.0) );
        tf_listener->transformPose(TargetFrameName,p,startPose);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      nav_msgs::Path nu;
      return nu;
      ros::Duration(1.0).sleep();
    }
    start.row = startPose.pose.position.y / cell_size;
    start.col = startPose.pose.position.x / cell_size;

    Node goal;
    goal.row = goalRow;
    goal.col = goalCol;
    PathFinder pf(start,goal);
    nav_msgs::Path originalPath = pf.getPath();
    nav_msgs::Path simplePath = simpilifyPath(originalPath);
    return simplePath;
}

bool GetPath(path_planning::GetPath::Request  &req, path_planning::GetPath::Response &res)
{
    res.path = servicePath(req.goal);
    return true;
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
    //ros::Subscriber path_sub_map = handle.subscribe<nav_msgs::OccupancyGrid>("/map",5,setMap);

    ros::Publisher path_pub = handle.advertise<nav_msgs::Path>("/Astar/path",10);


    //Wait 8 s for the map service.
    if(!ros::service::waitForService("/map_node/get_map", 8000))
    {
        ROS_ERROR("Map service unreachable.");
        return -1;
    }

    map_client = new ros::ServiceClient();
    *map_client = handle.serviceClient<map_tools::GetMap>("/map_node/get_map");
    updateMap();
    ros::ServiceServer path_srv = handle.advertiseService("/Astar/pathTest", GetPath);
    Node start;
    Node goal;

//    start.row = startX;
//    start.col = startY;
//    goal.row = goalX;
//    goal.col = goalY;

    //row stands for the y-coordinate; col stands for the x value;
    /*
    //TF package to set the start point
    std::string TargetFrameName = "/map";
    std::string CurrentFrame = "/base_link";

    geometry_msgs::PoseStamped p;
    p.header.frame_id = CurrentFrame;
    p.header.stamp = ros::Time(0);
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 1.0;

    try
    {
        tf_listener->waitForTransform(TargetFrameName, CurrentFrame, ros::Time(0), ros::Duration(1.0) );
        tf_listener->transformPose(TargetFrameName,p,startPose);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      return -1;
      ros::Duration(1.0).sleep();
    }
    start.row = startPose.pose.position.y * 100;
    start.col = startPose.pose.position.x * 100;
    */
    start.row = 20;//25;//92;//20;
    start.col = 20;//195;//200;//38;//193;//20;
    // row & col : start -> 211,195  || goal : 73,186
    //row:90, col:36 is the one we couldn't find the path
    goal.row = 25;//216;//216;//211;//90;//73;//25;//20;//40;
    goal.col = 50;//186;//104;//200;//36;//186;//200;//220;//20;
    PathFinder pf(start,goal);

    ros::Rate loop_rate(1);

    nav_msgs::Path path;
    nav_msgs::Path simpilifiedPath;
	while (ros::ok())
	{
        updateMap();

        if(map->data.size() < 1000)
        {
            ROS_INFO("Map is not yet intialized");
        }
        else
        {
          ROS_INFO("Planning a path...");
          path = pf.getPath();
          //std::cout<<"path size : "<<path.poses.size()<<std::endl;
          if(path.poses.size() > 0)
          {
             simpilifiedPath = simpilifyPath(path);
          }
          //path_pub.publish(simpilifiedPath);
          path_pub.publish(path);
        }
        ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
