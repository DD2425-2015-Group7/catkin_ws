#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

#include <mutex>

#define PI 3.14159

std::string odomFrame, baseFrame, mapFrame;

std::mutex mtx;
struct Poses{
    double x0 = 0.0, y0 = 0.0;
    double x = 0.0, y = 0.0, th = 0.0;
};
struct Poses coords;
ros::Time current_time;

tf::TransformBroadcaster *tf_broadcaster;
tf::TransformListener *tf_listener;

void updateOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    mtx.lock();
    current_time = msg->header.stamp;
    mtx.unlock();
    
}

void runMonteCarlo(void)
{
    coords.x = 0.0;
    coords.y = 0.0;
    coords.th = 0.0;
}

void publishTransform(void)
{
     //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped map_trans;
    mtx.lock();
    map_trans.header.stamp = current_time;
    mtx.unlock();
    map_trans.header.frame_id = mapFrame;
    map_trans.child_frame_id = odomFrame;

    map_trans.transform.translation.x = coords.x + coords.x0;
    map_trans.transform.translation.y = coords.y + coords.y0;
    map_trans.transform.translation.z = 0.0;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(coords.th);
    map_trans.transform.rotation = quat;

    //send the transform
    tf_broadcaster->sendTransform(map_trans);
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
	ros::init(argc, argv, "mcl_node");
	ros::NodeHandle n("/mcl_node");;
	
    n.param<std::string>("map_frame", mapFrame, "map");
    n.param<double>("map_offset_x", coords.x0, 0.0);
    n.param<double>("map_offset_y", coords.y0, 0.0);
    n.param<std::string>("odometry_frame", odomFrame, "odom");
    n.param<std::string>("robot_base_link", baseFrame, "base_link");

    tf::TransformBroadcaster broadcaster_obj;
    tf_broadcaster = &broadcaster_obj;
    tf::TransformListener listener_obj;
    tf_listener = &listener_obj;
    
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 200, updateOdom);

    current_time = ros::Time::now();

    const int rate = 10;
    ros::Rate loop_rate(rate);
    
	while (ros::ok())
	{
        runMonteCarlo();
        publishTransform();
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
