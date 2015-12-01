#include "ros/ros.h"
#include "math.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include <tf/transform_listener.h>

#include "tf/tf.h"

#include "motion_controllers/GetPathPoints.h"
#include "std_msgs/Bool.h"

//This closeEnough is used to determine when to move to next point
double closeEnough = 0.05;

ros::Publisher *pub_pose;
ros::Publisher *pub_espeak;
nav_msgs::Path path;

tf::TransformListener *tf_listener;

std::string TargetFrameName;
geometry_msgs::PoseStamped goalPose;
std_msgs::String str;

int nextPoint = 0;

bool GetPathPoints(motion_controllers::GetPathPoints::Request  &req, motion_controllers::GetPathPoints::Response  &res )
{
  nextPoint = 0;
  path.header = req.path.header;
  path.poses = req.path.poses;
  return true;
}

void calculatePosition(const nav_msgs::Odometry::ConstPtr& msg)
{
    static bool lastStopped = false, stopped = true;
    if(path.poses.size() < 1)
    {
        goalPose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        goalPose.pose.position.x = 0.0;
        goalPose.pose.position.y = 0.0;
        stopped = true;
        if(stopped == true && lastStopped == true){

        }else{
            pub_pose->publish(goalPose.pose);
        }
        lastStopped = stopped;
        return;
    }
    if(nextPoint > path.poses.size() - 1)
    {
        ROS_INFO("Destination reached");
        //set velocity to zero
        goalPose.pose.position.x = 0.0;
        goalPose.pose.position.y = 0.0;
        stopped = true;
    }else{
      stopped = false;
      geometry_msgs::PoseStamped p;
      p.header = path.header;
      p.header.stamp = ros::Time(0);
        p.pose = path.poses[nextPoint].pose;
	
        tf::StampedTransform transform;
        try
        {
            tf_listener->waitForTransform(TargetFrameName, p.header.frame_id, p.header.stamp, ros::Duration(1.0) );
            tf_listener->transformPose(TargetFrameName,p,goalPose);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
            ros::Duration(1.0).sleep();
        }

        double x = goalPose.pose.position.x;
        double y = goalPose.pose.position.y;
        //If not the last path pose, do not perform final rotation.
        if(nextPoint < path.poses.size() - 1){
            goalPose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        }
        double distance = sqrt(x*x + y*y);
        if(distance < closeEnough)
        {
            goalPose.pose.position.x = 0.0;
            goalPose.pose.position.y = 0.0;
            nextPoint++;
        }
    }

    ROS_INFO("Goal Point X :%f",goalPose.pose.position.x);
    ROS_INFO("Goal Point Y :%f",goalPose.pose.position.y);
    
    if(!(stopped && lastStopped)) {
      pub_pose->publish(goalPose.pose);
    }
    lastStopped = stopped;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_points_list");
    ros::NodeHandle handle;

    ros::Subscriber sub_odo = handle.subscribe<nav_msgs::Odometry>("/odom",1000,calculatePosition);
    ros::Publisher pub_pose_obj = handle.advertise<geometry_msgs::Pose>("/path_pose", 1000);

    ros::ServiceServer getPathPoints_server = handle.advertiseService("/motion_controllers/PathPointsExec", GetPathPoints);

    pub_pose = &pub_pose_obj;

    TargetFrameName = "/base_link";

    tf_listener = new tf::TransformListener();

    ros::Rate loopRate(20);

    while(ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}
