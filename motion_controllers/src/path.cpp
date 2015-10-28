#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "path");

  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 1000);

  ros::Rate loop_rate(10);
  geometry_msgs::PoseStamped p1,p2;
  //geometry_msgs::Point po1;
  p1.pose.position.x = 0.1;
  p1.pose.position.y = 0.2;

  p2.pose.position.x = 0.3;
  p2.pose.position.y = 0.4;
  nav_msgs::Path path;

  path.poses.push_back(p1);
  path.poses.push_back(p2);

  while (ros::ok())
  {
    path_pub.publish(path);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
