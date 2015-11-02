#include "ros/ros.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

#include <mutex>

#define PI 3.14159


double th0 = 0.0;
const double wheelDiameter = 0.099, wheelDist = 0.215, wheelRadius = wheelDiameter/2.0;
const int ticksPerRevolution = 360;
const double encStep = (wheelDiameter * PI)/(double)ticksPerRevolution;

std::string odomFrame, baseFrame;

std::mutex mtx;
int encL = 0, encR = 0, deltaEncL = 0, deltaEncR = 0;
double x = 0.0, y = 0.0, th = 0.0;
double vx = 0.0, vy = 0.0, vth = 0.0;
ros::Time current_time, last_time;

ros::Publisher *odom_pub;
tf::TransformBroadcaster *odom_broadcaster;

geometry_msgs::TransformStamped *odom_trans;
nav_msgs::Odometry *odom;

void poseUpdate(void);
void posePublish(void);
void updateEncoders(const ras_arduino_msgs::Encoders::ConstPtr& msg);

void updateEncoders(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    mtx.lock();
	encR += msg->delta_encoder1;
	encL += msg->delta_encoder2;
    deltaEncR = msg->delta_encoder1;
    deltaEncL = msg->delta_encoder2;
	mtx.unlock();
    poseUpdate();
}

void poseUpdate(void)
{
    mtx.lock();
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    
    double distance = (double)deltaEncR*encStep + (double)deltaEncL*encStep;
    distance /= 2.0;
    double delta_th = ((double)deltaEncR / wheelDist -  (double)deltaEncL  / wheelDist) * encStep;
    th = th0 + ((double)encR / wheelDist - (double)encL / wheelDist) * encStep;
    th = fmod(th, PI * 2.0);

    ROS_DEBUG("delta_th %f th %f dist %f\n", delta_th, th, distance);

    double delta_x = distance * cos(th);
    double delta_y = distance * sin(th);
    vx = delta_x/dt;
    vy = delta_y/dt;
    vth =  delta_th/dt;
    x += delta_x;
    y += delta_y;
    
    ROS_DEBUG("x %f y %f th %f\n", x, y, th);

    last_time = current_time;
    
    mtx.unlock();
}

void posePublish(void)
{
    mtx.lock();
    
    //first, we'll publish the transform over tf
    odom_trans->header.stamp = current_time;
    odom_trans->header.frame_id = odomFrame;
    odom_trans->child_frame_id = baseFrame;
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom_trans->transform.translation.x = x;
    odom_trans->transform.translation.y = y;
    odom_trans->transform.translation.z = 0.0;
    odom_trans->transform.rotation = odom_quat;

    //next, we'll publish the odometry message over ROS
    odom->header.stamp = current_time;
    odom->header.frame_id = odomFrame;

    //set the position
    odom->pose.pose.position.x = x;
    odom->pose.pose.position.y = y;
    odom->pose.pose.position.z = 0.0;
    odom->pose.pose.orientation = odom_quat;

    //set the velocity
    odom->child_frame_id = baseFrame;
    odom->twist.twist.linear.x = vx;
    odom->twist.twist.linear.y = vy;
    odom->twist.twist.angular.z = vth;
    
    mtx.unlock();
    
    //send the transform
    odom_broadcaster->sendTransform(*odom_trans);
    
    //publish the message
    odom_pub->publish(*odom);
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
	ros::init(argc, argv, "odometry");
	ros::NodeHandle n("/odometry");;
	
    n.param<std::string>("odometry_frame", odomFrame, "odom");
    n.param<std::string>("robot_base_link", baseFrame, "base_link");

    tf::TransformBroadcaster odom_broadcaster_obj;
    odom_broadcaster = &odom_broadcaster_obj;
    ros::Publisher odom_pub_obj = n.advertise<nav_msgs::Odometry>("odom", 50);
    odom_pub = &odom_pub_obj;
	ros::Subscriber encoder_sub = n.subscribe<ras_arduino_msgs::Encoders>("/arduino/encoders", 1000, updateEncoders);
    
    geometry_msgs::TransformStamped ot;
    odom_trans = &ot;
    
    nav_msgs::Odometry od;
    odom = &od;
    

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    const int rate = 20;
    ros::Rate loop_rate(rate);
    
	while (ros::ok())
	{
        posePublish();
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
