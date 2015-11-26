#include "ros/ros.h"
#include "math.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include <tf/transform_listener.h>

#include "tf/tf.h"


//This closeEnough is used to determine when to move to next point
double closeEnough = 0.05;

ros::Publisher *pub_pose;
ros::Publisher *pub_espeak;
nav_msgs::Path path;

tf::TransformListener *tf_listener;

std::string TargetFrameName;
geometry_msgs::PoseStamped goalPose;
std_msgs::String str;

void setPath(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("set path");
    path.header = msg->header;
    path.poses = msg->poses;
}


void calculatePosition(const nav_msgs::Odometry::ConstPtr& msg)
{
  // TODO: when we need to follow another path, "nextpoint" shoud be refreshed (set) to zero
  static int nextPoint = 0;

  if(path.poses.size() < 1)
  {
    return;
  }
  //ROS_INFO("calculatePosition");
  if(nextPoint > path.poses.size() - 1)
  {

      str.data = "destination reached";
      pub_espeak->publish(str);
    ROS_INFO("Destination reached");
    //set velocity to zero
    goalPose.pose.position.x = 0.0;
    goalPose.pose.position.y = 0.0;
  }
  else
  {
      geometry_msgs::PoseStamped p;
      p.header = path.header;
      p.header.stamp = ros::Time(0);
      p.pose = path.poses[nextPoint].pose;

      tf::StampedTransform transform;
      try
      {
          tf_listener->waitForTransform(TargetFrameName, p.header.frame_id, p.header.stamp, ros::Duration(1.0) );
          //tf_listener->lookupTransform(TargetFrameName, p.header.frame_id, ros::Time(0), transform);
          tf_listener->transformPose(TargetFrameName,p,goalPose);
          //transform.transformPose(TargetFrameName,p,goalPose);
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
    // rostopic pub /espeak/string std_msgs/String "Hi"
    str.data = "go to next point";
    pub_espeak->publish(str);
    //ROS_INFO("NextPoint");
   }
  }

  ROS_INFO("Goal Point X :%f",goalPose.pose.position.x);
  ROS_INFO("Goal Point Y :%f",goalPose.pose.position.y);
  //TargetPose
  pub_pose->publish(goalPose.pose);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_points_list");
    ros::NodeHandle handle;

    ros::Subscriber sub_path = handle.subscribe<nav_msgs::Path>("path_planner/path",1000,setPath);
    //ros::Subscriber sub_path = handle.subscribe<nav_msgs::Path>("/Astar/path",1000,setPath);

    ros::Subscriber sub_odo = handle.subscribe<nav_msgs::Odometry>("/odom",1000,calculatePosition);
    ros::Publisher pub_pose_obj = handle.advertise<geometry_msgs::Pose>("/path_pose", 1000);
    ros::Publisher pub_espeak_obj = handle.advertise<std_msgs::String>("/espeak/string",1000);


    pub_espeak = &pub_espeak_obj;
    pub_pose = &pub_pose_obj;

    TargetFrameName = "/base_link";

    tf_listener = new tf::TransformListener();


    ros::Rate loopRate(10);


    while(ros::ok())
    {

        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}
