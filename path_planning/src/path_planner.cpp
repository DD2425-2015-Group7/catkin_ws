#include "ros/ros.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "math.h"

#define PI 3.14159


ros::Publisher *path_pub;
tf::TransformListener *tf_listener;

nav_msgs::Path loadPath(std::string bagFile, std::string poseTopic)
{
    nav_msgs::Path path;
    
    rosbag::Bag bag;
    bag.open(bagFile, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(poseTopic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::PoseStamped::ConstPtr p = m.instantiate<geometry_msgs::PoseStamped>();
        if (p == NULL)
            continue;
        path.poses.push_back(*p);
            
    }
    bag.close();
    assert(path.poses.size()>0);
    path.header.stamp = ros::Time::now();
    path.header.frame_id = path.poses[0].header.frame_id;
    return path;
    
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
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle n("/path_planner");
    
    std::string bagFile;
	
    n.param<std::string>("bag_file", bagFile, "");

    tf::TransformListener tf_listener_obj;
    tf_listener = &tf_listener_obj;
    ros::Publisher path_pub_obj = n.advertise<nav_msgs::Path>("/path_planner/path", 2);
    path_pub = &path_pub_obj;
    
    const int rate = 4;
    ros::Rate loop_rate(rate);
    
    nav_msgs::Path path = loadPath(bagFile, "/move_base_simple/goal");

	while (ros::ok())
	{
        path_pub->publish(path);
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}

	return 0;
}
