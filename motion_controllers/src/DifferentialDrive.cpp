#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/Twist.h"
#include "math.h"

#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"

double distance;
double safetyDistance = 0.3;
double angle;

double safetyAngle = 0.5;

double MAX_LINEAR_VEL = 0.3;
double MAX_ANGULAR_VEL = 0.1;


ros::Publisher *pub_twist;
double orientati;

double angleDifference;

void setPosition(const geometry_msgs::Pose::ConstPtr& msg)
{
    distance = sqrt(msg.get()->position.x * msg.get()->position.x + msg.get()->position.y * msg.get()->position.y);
    //orientati = tf::getYaw(msg.get()->orientation);
    orientati = msg.get()->orientation.z;
    if(msg.get()->position.y > 0.001 ||  msg.get()->position.y < -0.001)
    {
        angle = atan2(msg.get()->position.y, msg.get()->position.x);
        angleDifference = abs(angle) - orientati;
    }
    else
    {
        angle = 0.0;
        angleDifference = 0.0;
    }


}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "DifferentialDrive");
    ros::NodeHandle handle;
    ros::Subscriber sub_posi = handle.subscribe("/path_pose", 1000, setPosition);

    ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    //ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);
    geometry_msgs::Twist t;
    ros::Rate loopRate(10);

    while(ros::ok())
    {
        if(angleDifference > safetyAngle)
        {
            t.linear.x = 0.0;
            t.angular.z = MAX_ANGULAR_VEL;
        }
        else
        {
            t.angular.z = 0.0;
            t.linear.x = MAX_LINEAR_VEL;
        }

        pub_twist.publish(t);
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}
