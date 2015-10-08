#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/Twist.h"
#include "math.h"

double distance;
double safetyDistance = 0.5;
double angle;

double MAX_LINEAR_VEL = 0.1;
double MAX_ANGULAR_VEL = 0.2;

void setPosition(const geometry_msgs::Point::ConstPtr& msg)
{
    distance = sqrt(msg.get()->x * msg.get()->x + msg.get()->y * msg.get()->y);

    if(msg.get()->y != 0)
    {
        angle = (msg.get()->x / msg.get()->y) ;
    }
    else
    {
        angle = 0.0;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "go_to_point");
    ros::NodeHandle handle;

    ros::Subscriber sub_posi = handle.subscribe("/next_point", 1000, setPosition);
    
    ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    geometry_msgs::Twist t;
    ros::Rate loopRate(10);

    while(ros::ok())
    {
        if(distance < safetyDistance)
        {
            t.linear.x = 0.0;
            t.angular.z = 0.0;
        }
        else
        {
            t.linear.x = MAX_LINEAR_VEL *  (1 - safetyDistance / distance);

            if(angle == 0.0)
            {
                t.angular.z = 0.0;
            }
            else
            {
                if(angle < 1.0 && angle > 0.0)
                {
                    t.angular.z = -MAX_ANGULAR_VEL;
                }
                else if(angle < 0.0 && angle > -1.0)
                {
                    t.angular.z = MAX_ANGULAR_VEL;
                }
                else
                {
                    t.angular.z =-MAX_ANGULAR_VEL / angle;
                }
            }
        }

        pub_twist.publish(t);
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}
