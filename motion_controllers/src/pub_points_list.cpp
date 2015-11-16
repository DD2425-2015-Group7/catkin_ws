#include "ros/ros.h"
#include "math.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_listener.h>

#include "tf/tf.h"

double closeEnough = 0.35;

ros::Publisher *pub_point;
nav_msgs::Path path;



void setPath(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("set path");
    path.header = msg->header;
    path.poses = msg->poses;
}


void calculatePosition(const nav_msgs::Odometry::ConstPtr& msg)
{
  double currentX;
  double currentY;
  double goalX;
  double goalY;
  double distance;

  geometry_msgs::Pose p;

  //Is this odom.orientation able to represent the current orientation?
  p.orientation.z = msg->pose.pose.orientation.z;
  // TODO: when we need to follow another path, "nextpoint" shoud be refreshed (set) to zero
  static int nextPoint = 0;

  if(path.poses.size() < 1)
  {
    return;
  }
  ROS_INFO("calculatePosition");
  if(nextPoint > path.poses.size() - 1)
  {
    ROS_INFO("Destination reached");
    //set velocity to zero
    p.position.x = 0.0;
    p.position.y = 0.0;
  }
  else
  {
    goalX = path.poses[nextPoint].pose.position.x;
    goalY = path.poses[nextPoint].pose.position.y;
    //ROS_INFO("Goal");
    currentX = msg->pose.pose.position.x; 
    currentY = msg->pose.pose.position.y;
    //ROS_INFO("Current");
   double distance = sqrt((goalY-currentY)*(goalY-currentY) + (goalX-currentX)*(goalX-currentX));
   if(distance > closeEnough)    
   {
       std::cerr << "Goal X: " <<goalX << std::endl;
       std::cerr << "Goal Y: " <<goalY   << std::endl;
       std::cerr << "Current X: " <<currentX << std::endl;
       std::cerr << "Current Y: " <<currentY   << std::endl;
       p.position.x = goalX - currentX;
       p.position.y = goalY - currentY;
       std::cerr << "point X: " <<p.position.x << std::endl;
       std::cerr << "point Y: " <<p.position.y << std::endl;

       //std::cerr << "Quterniaon: " << p.orientation.z << std::endl;
   }
   else
   {
    p.position.x = 0.0;
    p.position.y = 0.0;

    nextPoint++;

    ROS_INFO("NextPoint:");     
   }
  }
  pub_point->publish(p);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_points_list");
    ros::NodeHandle handle;

    ros::Subscriber sub_path = handle.subscribe<nav_msgs::Path>("path_planner/path",1000,setPath);
    //ros::Subscriber sub_path = handle.subscribe<nav_msgs::Path>("/path",1000,setPath);
    ros::Subscriber sub_odo = handle.subscribe<nav_msgs::Odometry>("/odom",1000,calculatePosition);
    ros::Publisher pub_point_obj = handle.advertise<geometry_msgs::Pose>("/path_pose", 1000);
    pub_point = &pub_point_obj;

    //ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
/*
    tf::TransformListener listener;
    tf::StampedTransform transform;

    //geometry_msgs::Point poi;
    geometry_msgs::PointStamped poi;
//    poi.x = 1.0;
//    poi.y = 0.5;
    poi.point.x = 1.0;
    poi.point.y = 0.5;
        //    poi->pose.position.x = 1.0;
        //    poi->pose.position.y = 0.5;
*/
    
    ros::Rate loopRate(10);
    while(ros::ok())
    {/*
        try
        {
          listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform("/base_link","/map",ros::Time(0),transform);
          listener.transformPoint("/base_link,",ros::Time(0),poi,"/map");
          // listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
          // listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
        }
        catch(tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
//        geometry_msgs:: after;
//        after.x = transform.;
//        after.y = transform.getOrigin().y();

        pub_point_obj.publish(poi);
        */
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}

