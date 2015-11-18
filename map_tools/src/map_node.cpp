#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "math.h"
#include "map_tools/MapStorage.h"

#include "map_tools/AddEllipse.h"
#include "map_tools/GetMap.h"

std::string mapFrame, wallFile;
double wallThickness, inflationRadius, cellSize;
int mapWidth, mapHeight;

MapStorage *ms;

ros::Time current_time;
tf::TransformBroadcaster *tf_broadcaster;
tf::TransformListener *tf_listener;

bool addEllipse(map_tools::AddEllipse::Request  &req,
         map_tools::AddEllipse::Response &res)
{
    ms->addEllipse(req.x, req.y, req.a, req.b, req.th);
    ms->renderGrid();
    return true;
}

bool getMap(map_tools::GetMap::Request  &req,
         map_tools::GetMap::Response &res)
{
    if(req.type.data.compare("default") == 0){
        ms->getMap("default", res.map);
        return true;
    }
    if(req.type.data.compare("inflated") == 0){
        ms->getMap("inflated", res.map);
        return true;
    }
    if(req.type.data.compare("distance") == 0){
        ms->getMap("distance", res.map);
        return true;
    }
    return false;
}

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
    n.param<std::string>("wall_file", wallFile, "");
    n.param<double>("wall_thickness", wallThickness, 0.02);
    n.param<double>("obstacle_inflation_radius", inflationRadius, 0.0);
    n.param<double>("cell_size", cellSize, 0.02);
    n.param<int>("map_width_cells", mapWidth, 500);
    n.param<int>("map_height_cells", mapHeight, 500);

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = mapFrame;
    map.header.stamp = current_time;
    ros::Publisher map_pub_obj = n.advertise<nav_msgs::OccupancyGrid>("/map", 2);
    ms = new MapStorage(mapWidth, mapHeight, cellSize, 100, inflationRadius);

    ms->loadWalls(wallFile, wallThickness);
    ms->renderGrid();

    ros::ServiceServer ellipse_srv = n.advertiseService("add_ellipse", addEllipse);
    ros::ServiceServer map_srv = n.advertiseService("get_map", getMap);

    tf::TransformBroadcaster broadcaster_obj;
    tf_broadcaster = &broadcaster_obj;
    tf::TransformListener listener_obj;
    tf_listener = &listener_obj;

    current_time = ros::Time::now();

    const int rate = 10, mapRate = 1;
    int counter = 0;
    ros::Rate loop_rate(rate);

	while (ros::ok())
	{
        if(counter < rate/mapRate){
            counter++;
        }else{
            ms->getMap("inflated", map);
            map_pub_obj.publish(map);
            counter = 0;
        }
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
