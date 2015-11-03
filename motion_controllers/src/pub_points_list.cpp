#include "ros/ros.h"
#include "math.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include "tf/transform_listener.h"


int nextPoint = 0;
double closeEnough = 0.2;

double currentX;
double currentY;
double goalX;
double goalY;

double distance;

//geometry_msgs::Twist t;
geometry_msgs::Point p;
nav_msgs::Path path;

//tf::TransformListener listener;


void setPath(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("set path");
    path = *msg;
}

// void calculatePosition(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     goalX = path.poses[nextPoint].pose.position.x;
//     goalY = path.poses[nextPoint].pose.position.y;

//     currentX = msg->pose.pose.position.x;
//     currentY = msg->pose.pose.position.y;

//    double distance = sqrt((goalY-currentY)*(goalY-currentY) + (goalX-   currentX)*(goalX-currentX));
//    if(distance < closeEnough)
//    {
//        if(nextPoint > path.poses.size() - 1)
//        {
//            ROS_INFO("Destination reached");
//            //set velocity to zero
//            p.x = 0.0;
//            p.y = 0.0;
//        }
//        else
//        {
//            nextPoint++;
//        }
//    }
//    else
//    {
//        p.x = goalX - currentX;
//        p.y = goalY - currentY;
//    }
// }

void calculatePosition(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("calculatePosition");
  if(nextPoint > path.poses.size() - 1)
  {
    ROS_INFO("Destination reached");
    //set velocity to zero
    p.x = 0.0;
    p.y = 0.0;    
  }
  else
  {

    goalX = path.poses[nextPoint].pose.position.x;
    goalY = path.poses[nextPoint].pose.position.y;
    //ROS_INFO("Goal Position X:" , goalX , "Goal Position Y" , goalY);
    ROS_INFO("Goal");
    currentX = msg->pose.pose.position.x;
    currentY = msg->pose.pose.position.y;
    //ROS_INFO("Current Position X:", currentX ,"Current Position Y",currentY);
    ROS_INFO("Current");
   double distance = sqrt((goalY-currentY)*(goalY-currentY) + (goalX-currentX)*(goalX-currentX));
    //ROS_INFO("distance",distance);
   if(distance > closeEnough)    
   {
       p.x = goalX - currentX;
       p.y = goalY - currentY;    
   }
   else
   {
    p.x = 0.0;
    p.y = 0.0; 
    nextPoint++;     
   }
  }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_points_list");
    ros::NodeHandle handle;

    //ros::Subscriber sub_path = handle.subscribe<nav_msgs::Path>("path_planner/path",1000,setPath);
    ros::Subscriber sub_path = handle.subscribe<nav_msgs::Path>("/path",1000,setPath);
    ros::Subscriber sub_odo = handle.subscribe<nav_msgs::Odometry>("odom",1000,calculatePosition);
    ros::Publisher pub_point = handle.advertise<geometry_msgs::Point>("/path_point", 1000);
    //ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    ros::Rate loopRate(10);

    while(ros::ok())
    { 
        ROS_INFO("Running");
        pub_point.publish(p);
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}

