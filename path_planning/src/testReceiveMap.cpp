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

nav_msgs::OccupancyGrid *map;


void setMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
      map->data = msg.get()->data;
      map->info = msg.get()->info;
      map->header = msg.get()->header;
      std::cout<< "Setting the map" <<std::endl;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testReceiveMap");
    ros::NodeHandle handle;

    map = new nav_msgs::OccupancyGrid();

    map->info.height = 0;
    map->info.width = 0;

    ros::Subscriber path_sub_map = handle.subscribe<nav_msgs::OccupancyGrid>("/map",5,setMap);

//    start.row = 10;
//    start.col = 12;
//    goal.row = 100;
//    goal.col = 100;

    std::cout<<"Map array length1: " << map->data.size() << std::endl;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if(map->data.size() < 10)
        {
            ROS_INFO("Map not received");
        }
        else
        {
            ROS_INFO("Map received");
            if(isCanMove(100,120))
            {
                 ROS_INFO("Passable");
            }
            else
            {
                 ROS_INFO("Not passable");
            }
        }
        //std::cout<< "Map_info: "<< map->info << std::endl;
        //  std::cout<<"Map array length2: " << map->data.size() << std::endl;
        //ROS_INFO("Hello");

        ros::spinOnce(); // Run the callbacks.
        loop_rate.sleep();
    }

    return 0;
}
