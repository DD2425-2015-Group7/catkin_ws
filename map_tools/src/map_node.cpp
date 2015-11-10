#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "math.h"
#include "map_tools/MapStorage.h"

#include <mutex>

#define PI 3.14159

std::string mapFrame;
double wallThickness, inflationRadius, cellSize;
int mapWidth, mapHeight;

ros::Time current_time;

tf::TransformBroadcaster *tf_broadcaster;
tf::TransformListener *tf_listener;


int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "map_node");
	ros::NodeHandle n("/map_node");;
	
    n.param<std::string>("map_frame", mapFrame, "map");
    n.param<double>("wall_thickness", wallThickness, 0.02);
    n.param<double>("obstacle_inflation_radius", inflationRadius, 0.0);
    n.param<double>("cell_size", cellSize, 0.02);
    n.param<int>("map_width_cells", mapWidth, 500);
    n.param<int>("map_height_cells", mapHeight, 500);

    ros::Publisher map_pub_obj = n.advertise<nav_msgs::OccupancyGrid>("/map", 2);
    MapStorage ms(mapWidth, mapHeight, cellSize, 100);

    ms.addWall(0.3, 0.2, 1.0, 1.5, wallThickness);
    ms.addEllipse(2.0, 2.5, 0.4, 0.2, 0.5);
    ms.renderGrid();


    tf::TransformBroadcaster broadcaster_obj;
    tf_broadcaster = &broadcaster_obj;
    tf::TransformListener listener_obj;
    tf_listener = &listener_obj;

    current_time = ros::Time::now();

    const int rate = 1;
    ros::Rate loop_rate(rate);

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = mapFrame;
    map.header.stamp = current_time;
    
	while (ros::ok())
	{
        ms.getMap(map);
        map_pub_obj.publish(map);
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
