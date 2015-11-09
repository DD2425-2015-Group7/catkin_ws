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
// Linear acceleration of x axis
double lin_acc_x = 0.0;
// Angular velocity of x axis
double ang_vel_z = 0.0;

bool start = false;
bool crash_lin = false;
bool crash_ang = false;
int count = 0;

void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)

{
    start = true;
    lin_acc_x = msg->linear_acceleration.x;
    if (fabs(lin_acc_x) < 1) {
        count++;
    } else {
        count = 0;
    }

    ang_vel_z= msg->angular_velocity.z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_crash");

    ros::NodeHandle n;

    ros::Subscriber imu_data_sub = n.subscribe("imu/data", 1000, imuDataCallback);

    ros::Publisher espeak_pub = n.advertise<std_msgs::String>("espeak/string", 1000);
    ros::Publisher crash_pub = n.advertise<std_msgs::String>("crash", 1000);

    ros::Rate loop_rate(loop_rate);

    ROS_INFO("loop_rate");

    std_msgs::String espeak_msg;

    while (ros::ok()) {
        std::cout << "Count: " << count << std::endl;
        if (start) {
            // if cmd_linear_velocity > epsl
            if (count > 250) {
                ROS_INFO("LINEAR CRASH");
                count = 250;
                crash_lin == true;
                espeak_msg.data = "LINEAR CRASH";
                espeak_pub.publish(espeak_msg);
                crash_pub.publish(espeak_msg);
            } else {
                crash_lin = false;
            }
            // fin du if
        }

        // if cmd_ang_veloc_z > epsa
        if (fabs(ang_vel_z) < 0.1) {
            crash_ang = true;
        } else {
            crash_ang = false;
            ROS_INFO("ANGULAR CRASH");
            espeak_msg.data = "ANGULAR CRASH";
            espeak_pub.publish(espeak_msg);
            crash_pub.publish(espeak_msg);
        }
        // fin du if

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0 ;
}



