#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

// Loop rate
int loop_rate = 100;
/* IMU data */
// Linear acceleration of y axis (y is the axis to go forward with the robot, in the IMU reference system, ie x in the robot reference system)
double imu_lin_acc_x = 0.0;
// Angular velocity of x axis
double imu_ang_vel_z = 0.0;

/* Velocities command data. Warning: the x axis in the robot reference is the y axis in the IMU one */
double lin_vel_x = 0.0;
double ang_vel_z = 0.0;

// Start boolean values to wait the data before beginning
bool start_imu = false;
bool start_vel = false;
// Boolean flags about the crashes
bool crash_lin = false;
bool crash_ang = false;

// Counter used to verify if the acceleration stays during a long time nul
int count_threshold = 250;
int count = 0;

// Limit absolute values to use to say that we want to move (from cmd_vel topic)
double lin_limit = 1.0;
double ang_limit = 0.5;

// Thresholds used to detect that the robot is stuck
double lin_acc_threshold = 0.25;
double ang_vel_threshold = 1.0;

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  start_imu = true;
  imu_lin_acc_x = msg->linear_acceleration.y;
  if (fabs(imu_lin_acc_x) < lin_acc_threshold) {
    count++;
  } else {
    count = 0;
  }

  imu_ang_vel_z= msg->angular_velocity.z;
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  start_vel = true;
  lin_vel_x = msg->linear.x;
  ang_vel_z = msg->angular.z;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_crash");

  ros::NodeHandle n;

  ros::Subscriber imu_data_sub = n.subscribe("imu/data", 1000, imuDataCallback);

  ros::Publisher espeak_pub = n.advertise<std_msgs::String>("espeak/string", 1000);
  ros::Publisher crash_pub = n.advertise<std_msgs::String>("crash", 1000);

  /* Subscription to the velocity node */
  ros::Subscriber twist_sub = n.subscribe("/cmd_vel", 1000, velocityCallback);

  ros::Rate loop_rate(loop_rate);

  ROS_INFO("loop_rate");

  std_msgs::String espeak_msg;

  while (ros::ok()) {
    std::cout << "Count: " << count << std::endl;
    if (start_imu || start_vel) {
      if (lin_vel_x > lin_limit) {
	if (count > count_threshold) {
	  ROS_INFO("LINEAR CRASH");
	  count = count_threshold;
	  crash_lin == true;
	  espeak_msg.data = "LINEAR CRASH";
	  espeak_pub.publish(espeak_msg);
	  crash_pub.publish(espeak_msg);
	} else {
	  crash_lin = false;
	}
      }
        

      if (ang_vel_z > ang_limit) {
	if (fabs(imu_ang_vel_z) < ang_vel_threshold) {
	  crash_ang = true;
	} else {
	  crash_ang = false;
	  ROS_INFO("ANGULAR CRASH");
	  espeak_msg.data = "ANGULAR CRASH";
	  espeak_pub.publish(espeak_msg);
	  crash_pub.publish(espeak_msg);
	}
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0 ;
}



