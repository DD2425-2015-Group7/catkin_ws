#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "math.h"
#include "map_tools/MapStorage.h"

#include "map_tools/AddEllipse.h"
#include "map_tools/AddObjects.h"
#include "map_tools/GetMap.h"
#include "classification/ClassifiedObjectArray.h"

std::string mapFrame, wallFile;
double wallThickness, inflationRadius, cellSize;

MapStorage *ms, *mso;
classification::ClassifiedObjectArray *clsObj;

ros::Time current_time;
tf::TransformBroadcaster *tf_broadcaster;
tf::TransformListener *tf_listener;

bool addEllipse(map_tools::AddEllipse::Request  &req,
         map_tools::AddEllipse::Response &res)
{
    ms->stackEllipse(req.x, req.y, req.a, req.b, req.th);
    ms->renderGrid();
    mso->stackEllipse(req.x, req.y, req.a, req.b, req.th);
    mso->renderGrid();
    return true;
}

void getObjectDims(classification::ClassifiedObject &co, double &a, double &b)
{
    //TODO: improve. Add also orientation.
    //TODO: draws a circle instead of an ellipse. Why?
    if(co.name.compare("debris") == 0){
        a = 0.06;
        b = 0.15;
    }else{
        a = 0.02;
        b = 0.02;
    }
}

bool inTolerance(double a, double b, double tol)
{
    if(a < b - tol)
        return false;
    if(a > b + tol)
        return false;
    return true;
}

void saveObject(classification::ClassifiedObject &co)
{
    for(int i = 0; i < clsObj->objects.size(); i++){
        if(clsObj->objects[i].id != co.id)
            continue;
        if(clsObj->objects[i].name.compare(co.name) != 0)
            continue;
        if(!inTolerance(clsObj->objects[i].p.x, co.p.x, 0.2))
            continue;
        if(!inTolerance(clsObj->objects[i].p.y, co.p.y, 0.2))
            continue;
        //TODO: some averaging? OK, maybe not.
        clsObj->objects[i] = co;
        return;
    }
    clsObj->objects.push_back(co);
}

bool addObjects(map_tools::AddObjects::Request  &req,
         map_tools::AddObjects::Response &res)
{
    for(int i = 0; i < req.array.objects.size(); i++){
        saveObject(req.array.objects[i]);
    }
    mso->clearEllipses();
    for(int i = 0; i < clsObj->objects.size(); i++){
        double x = clsObj->objects[i].p.x;
        double y = clsObj->objects[i].p.y;
        double a, b;
        getObjectDims(clsObj->objects[i], a, b);
        mso->stackEllipse(x, y, a, b, 0);
    }
    mso->renderGrid();
    return true;
}

bool getMap(map_tools::GetMap::Request  &req,
         map_tools::GetMap::Response &res)
{
    if(req.type.data.compare("default") == 0){
        res.mappedObjects = *clsObj;
        ms->getMap("default", res.map);
        return true;
    }
    if(req.type.data.compare("inflated") == 0){
        res.mappedObjects = *clsObj;
        ms->getMap("inflated", res.map);
        return true;
    }
    if(req.type.data.compare("distance") == 0){
        res.mappedObjects = *clsObj;
        ms->getMap("distance", res.map);
        return true;
    }
    if(req.type.data.compare("default_obj") == 0){
        res.mappedObjects = *clsObj;
        mso->getMap("default", res.map);
        return true;
    }
    if(req.type.data.compare("inflated_obj") == 0){
        res.mappedObjects = *clsObj;
        mso->getMap("inflated", res.map);
        return true;
    }
    if(req.type.data.compare("distance_obj") == 0){
        res.mappedObjects = *clsObj;
        mso->getMap("distance", res.map);
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

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = mapFrame;
    map.header.stamp = current_time;
    ros::Publisher map_pub_obj = n.advertise<nav_msgs::OccupancyGrid>("/map", 2, true);
    clsObj = new classification::ClassifiedObjectArray();
    ms = new MapStorage(cellSize, 100, 10, inflationRadius);
    mso = new MapStorage(cellSize, 100, 10, inflationRadius);

    ms->loadWalls(wallFile, wallThickness);
    ms->renderGrid();
    
    mso->loadWalls(wallFile, wallThickness);
    mso->renderGrid();

    ros::ServiceServer ellipse_srv = n.advertiseService("add_ellipse", addEllipse);
    ros::ServiceServer obj_srv = n.advertiseService("add_objects", addObjects);
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
            mso->getMap("inflated", map);
            map.header.stamp = current_time;
            map_pub_obj.publish(map);
            counter = 0;
        }
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
