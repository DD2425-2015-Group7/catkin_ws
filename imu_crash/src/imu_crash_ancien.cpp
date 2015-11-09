#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

// Loop rate
int loop_rate = 10;
// Acceleration and linear velocity of x axis
double lin_acc_x = 0;
double lin_vel_x = 0;

bool parle = false;

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    lin_acc_x = msg->linear_acceleration.x;
    ROS_INFO("IMU :%f", msg->linear_acceleration.x);
    if (lin_acc_x > 0.001) {
        parle = true;
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_crash");

  ros::NodeHandle n; 

  ros::Subscriber imu_data_sub = n.subscribe("imu/data", 1000, imuDataCallback);

  ros::Publisher espeak_pub = n.advertise<std_msgs::String>("espeak/string", 1000);

  ros::Rate loop_rate(loop_rate);

  ROS_INFO("loop_rate");



  while (ros::ok()) {
      if (parle) {
          std_msgs::String espeak_msg;
          std::stringstream ss;
          ss << "hello world " << 1;
          espeak_msg.data = ss.str();
          std::cout << "LIINNNNN" << lin_acc_x << std::endl;
          espeak_pub.publish(espeak_msg);
      } else {
          ROS_INFO("ELSE");
      }

      ros::spin();
      ROS_INFO("SORTIE");
      loop_rate.sleep();
  }
  std::cout << "MERDE";
  return 0 ;
}
  
