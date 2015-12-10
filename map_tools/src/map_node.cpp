#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "math.h"
#include "map_tools/MapStorage.h"

#include "map_tools/AddEllipse.h"
#include "map_tools/AddObjects.h"
#include "map_tools/ObjectStorage.h"
#include "map_tools/GetMap.h"
#include "classification/ClassifiedObjectArray.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

std::string mapFrame, wallFile;
double wallThickness, inflationRadius, cellSize;

//Current version of the map, increments with every load/add object call.
int version = 0;
MapStorage *ms, *mso;
classification::ClassifiedObjectArray *clsObj;

ros::Time current_time;
tf::TransformBroadcaster *tf_broadcaster;
tf::TransformListener *tf_listener;
ros::Publisher *vis_pub;

bool addEllipse(map_tools::AddEllipse::Request  &req,
         map_tools::AddEllipse::Response &res)
{
    ms->stackEllipse(req.x, req.y, req.a, req.b, req.th);
    ms->renderGrid();
    mso->stackEllipse(req.x, req.y, req.a, req.b, req.th);
    mso->renderGrid();
    return true;
}

void stackObject(classification::ClassifiedObject &co)
{
    //TODO: draws a circle instead of an ellipse. Why?
    double a = 0.02;
    double b = 0.02;
    double x = co.p.x;
    double y = co.p.y;
    mso->stackEllipse(x, y, a, b, 0);
}

void stackDebris(classification::ClassifiedObject &co)
{
    double w = 0.1;
    mso->stackLine(co.p.x, co.p.y, co.p2_debris.x, co.p2_debris.y, w);
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
    ROS_INFO("map_node Adding objects....");
    for(int i = 0; i < req.array.objects.size(); i++){
        saveObject(req.array.objects[i]);
    }
    
    mso->clearEllipses();
    mso->clearLines();
    for(int i = 0; i < clsObj->objects.size(); i++){
        if(clsObj->objects[i].name.compare("debris") == 0){
            ROS_INFO("map_node fake info: Adding debris");
            //stackDebris(clsObj->objects[i]);
        }else{
            ROS_INFO("map_node fake info: Adding object");
            //stackObject(clsObj->objects[i]);
        }
    }
    ROS_INFO("map_node rendering");
    mso->renderGrid();
    version += 1;
    return true;
}

classification::ClassifiedObjectArray loadObjects(std::string bagFile, std::string topic)
{
    classification::ClassifiedObjectArray ob;
    
    rosbag::Bag bag;
    bag.open(bagFile, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        classification::ClassifiedObjectArray::ConstPtr p = m.instantiate<classification::ClassifiedObjectArray>();
        if (p == NULL)
            continue;
        if(p->objects.size()>0){
            ob = *p;
            break;
        }
            
    }
    bag.close();
    return ob;
    
}

void storeObjects(std::string bagFile, std::string topic, classification::ClassifiedObjectArray ob)
{
    rosbag::Bag bag;
    bag.open(bagFile, rosbag::bagmode::Write);
    bag.write(topic, ros::Time::now(), ob); // The timestamp can be left out.
    bag.close();
}

bool objectStorage(map_tools::ObjectStorage::Request  &req,
         map_tools::ObjectStorage::Response &res)
{
    if(req.action.compare("load") == 0){
        ROS_INFO("map_node Loading objects...");
        *clsObj = loadObjects(req.bag_file, "/mapped_objects");
        version += 1;
    }else if(req.action.compare("store") == 0){
        ROS_INFO("map_node Storing objects...");
        storeObjects(req.bag_file, "/mapped_objects", *clsObj);
    }else{
        ROS_ERROR("map_node Invalid object storage action chosen!");
        return false;
    }
    return true;
}

void publishObjects(void)
{
    
    // 0 - object, 1 - red cube, 2 - red hollow cube, 3 - blue_cube,
    // 4 - green cube, 5 - yellow_cube, 6 - yellow_ball, 7 - red_ball
    // 8 - green_cylinder, 9 - blue_triangle, 10 - purple_cross,
    // 11 - purple_star, 12 - orange star (patric), 13 - debris
    // black, red cu, red cu, blue cu; green cu, yellow cu, yellow s, red s;
    // green ci, blue ci, purple ci, purple s, orange s. 
    // green (1,1,0), purple (1,0,1), orange (1, 0.6, 0)
    float coloursR[13] = {0.0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1};
    float coloursG[13] = {0.8, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0.6};
    float coloursB[13] = {0.8, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0};
    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker obj_marker;
    obj_marker.header.frame_id = mapFrame;
    obj_marker.header.stamp = ros::Time();
    obj_marker.ns = "objects";
    obj_marker.type = visualization_msgs::Marker::CUBE;
    obj_marker.action = visualization_msgs::Marker::ADD;
    obj_marker.scale.x = 0.05;
    obj_marker.scale.y = 0.05;
    obj_marker.scale.z = 0.05;
    obj_marker.color.a = 1.0;
    obj_marker.color.r = (255.0/255.0);
    obj_marker.color.g = (0.0/255.0);
    obj_marker.color.b = (0.0/255.0);
    obj_marker.pose.position.z = 0.05;
    
    for(int i = 0; i < clsObj->objects.size(); i++){
        int id = clsObj->objects[i].id;
        obj_marker.pose.position.x = clsObj->objects[i].p.x;
        obj_marker.pose.position.y = clsObj->objects[i].p.y;
        obj_marker.id = i;
        obj_marker.type = visualization_msgs::Marker::CUBE;
        if(clsObj->objects[i].name.compare("debris") == 0 || id == 13){
            obj_marker.color.r = (1.0);
            obj_marker.color.g = (1.0);
            obj_marker.color.b = (1.0);
        }else{
            assert(id >= 0 && id <= 12);
            obj_marker.color.r = coloursR[id];
            obj_marker.color.g = coloursG[id];
            obj_marker.color.b = coloursB[id];
        }
        if(id == 6 || id == 7 || id == 11 || id == 12){
            obj_marker.type = visualization_msgs::Marker::SPHERE;
        }else if(id == 8 || id == 9){
            obj_marker.type = visualization_msgs::Marker::CYLINDER;
        }
        all_markers.markers.push_back(obj_marker);
    }
    vis_pub->publish(all_markers);
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
    ros::Publisher map_version_pub = n.advertise<std_msgs::Int32>("/map_node/version", 2);
    vis_pub = new ros::Publisher();
    *vis_pub = n.advertise<visualization_msgs::MarkerArray>("/map_node/objects", 1 );
    clsObj = new classification::ClassifiedObjectArray();
    ms = new MapStorage(cellSize, 100, 10, inflationRadius);
    mso = new MapStorage(cellSize, 100, 10, inflationRadius);

    ms->loadWalls(wallFile, wallThickness);
    ms->renderGrid();
    
    mso->loadWalls(wallFile, wallThickness);
    mso->renderGrid();
    
    version += 1;

    ros::ServiceServer ellipse_srv = n.advertiseService("add_ellipse", addEllipse);
    ros::ServiceServer obj_srv = n.advertiseService("add_objects", addObjects);
    ros::ServiceServer obst_srv = n.advertiseService("object_storage", objectStorage);
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
        std_msgs::Int32 vmsg;
        vmsg.data = version;
        map_version_pub.publish(vmsg);
        publishObjects();
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
