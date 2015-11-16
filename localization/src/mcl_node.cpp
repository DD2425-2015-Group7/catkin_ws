#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"
#include "math.h"
#include "localization/MonteCarlo.h"

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

MonteCarlo *mc;
OdometryModel *om;
struct PoseState odomState;

void updateOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    mtx.lock();
    current_time = msg->header.stamp;
    odomState.set(0.0);
    odomState.x = msg->pose.pose.position.x;
    odomState.y = msg->pose.pose.position.y;
    odomState.yaw = tf::getYaw(msg->pose.pose.orientation);
    mtx.unlock();
    
}

void runMonteCarlo(void)
{
    mc->run(odomState);
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

void particles2PoseArray(const std::vector<MonteCarlo::StateW> &particles, geometry_msgs::PoseArray &pa)
{
    geometry_msgs::Pose p;
    pa.poses.clear();
    for(int i = 0; i<particles.size(); i++){
        p.position.x = particles[i].s.x;
        p.position.y = particles[i].s.y;
        p.position.z = particles[i].s.z;
        tf::quaternionTFToMsg(
            tf::createQuaternionFromRPY(particles[i].s.roll, particles[i].s.pitch, particles[i].s.yaw),
            p.orientation);
        pa.poses.push_back(p);
    }
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
    n.param<std::string>("robot_base_link", baseFrame, "base_link");
    n.param<double>("map_offset_x", coords.x0, 0.0);
    n.param<double>("map_offset_y", coords.y0, 0.0);
    
    double odom_a1, odom_a2, odom_a3, odom_a4;
    n.param<std::string>("odometry_frame", odomFrame, "odom");
    n.param<double>("odometry_model_a1", odom_a1, 0.05);
    n.param<double>("odometry_model_a2", odom_a2, 0.01);
    n.param<double>("odometry_model_a3", odom_a3, 0.03);
    n.param<double>("odometry_model_a4", odom_a4, 0.01);
    
    int nParticles, rate;
    double initConeRadius, initYawVar;
    n.param<int>("mcl_particles", nParticles, 200);
    n.param<int>("mcl_rate", rate, 5);
    n.param<double>("mcl_init_cone_radius", initConeRadius, 0.2);
    n.param<double>("mcl_init_yaw_variance", initYawVar, 0.5);

    tf::TransformBroadcaster broadcaster_obj;
    tf_broadcaster = &broadcaster_obj;
    tf::TransformListener listener_obj;
    tf_listener = &listener_obj;
    
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 50, updateOdom);
    ros::Publisher particle_pub_obj = n.advertise<geometry_msgs::PoseArray>("/mcl/particles", 5);
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = mapFrame;

    struct PoseState pose;
    pose.set(0.0);
    om = new OdometryModel(odom_a1, odom_a2, odom_a3, odom_a4);
    mc = new MonteCarlo(om, nParticles);
    mc->init(pose, initConeRadius, initYawVar);
    /*
    if(mc->test())
        ROS_INFO("MCL test passed.");
    else
        ROS_INFO("MCL test failed.");
    */
    current_time = ros::Time::now();

    ros::Rate loop_rate(rate);
    
	while (ros::ok())
	{
        runMonteCarlo();
        publishTransform();
        particles2PoseArray(mc->getParticles(), poseArray);
        particle_pub_obj.publish(poseArray);
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
