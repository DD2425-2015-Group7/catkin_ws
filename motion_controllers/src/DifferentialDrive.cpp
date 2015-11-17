#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/Twist.h"
#include "math.h"

#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"

double distance;
double safetyDistance = 0;
double angle;

double safetyAngle = 0.1;

double MAX_LINEAR_VEL = 0.2;
double MAX_ANGULAR_VEL = 1.5;


ros::Publisher *pub_twist;
double orientati;

enum States
{
    STOP,
    ROTATE_RIGHT,
    ROTATE_LEFT,
    FORWORD
};

States state;

void setPosition(const geometry_msgs::Pose::ConstPtr& msg)
{
    //distance = sqrt(msg.get()->position.x * msg.get()->position.x + msg.get()->position.y * msg.get()->position.y);
    //ROS_INFO("Distance :%f",distance);

    //As we use the tf package to tansform the point, so the distance will be just the x value.
    distance = msg.get()->position.x;

    //orientati = tf::getYaw(msg.get()->orientation);
    //orientati = msg.get()->orientation.z;

    if(msg.get()->position.y > 0.001 ||  msg.get()->position.y < -0.001)
    {
        angle = atan2(msg.get()->position.y, msg.get()->position.x);
    }
    else
    {
        angle = 0.0;
    }


    ROS_INFO("Angle :%f",angle);
    if(angle > safetyAngle)
    {
        ROS_INFO("Change state to right!!!!");
        state = ROTATE_LEFT;
    }
    else if(angle < -safetyAngle)
    {
        ROS_INFO("change state to left!!!!");
        state = ROTATE_RIGHT;
    }
    else if (distance > safetyDistance)
    {
        ROS_INFO("move forward!!!!");
        state = FORWORD;
    }
    else
    {
        ROS_INFO("STOP!!!!!!");
        state = STOP;
    }
}

double smoothUpdateVelocity(double current, double required, double step)
{
    double diff = current - required;
    if(fabs(diff) > 1.5*step)
    {
       current -= (diff > 0) ? step : -step;
    }
    else
    {
        current = required;
    }
    return current;
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
        switch(state)
        {
        case(ROTATE_RIGHT):
            //rotate to the right direction


            //t.linear.x = smoothUpdateVelocity(t.linear.x,0,0.1);
            t.angular.z = smoothUpdateVelocity(t.angular.z, -MAX_ANGULAR_VEL,0.05);
            std::cerr<< "Rotate to Right" <<std::endl;


            t.linear.x = 0;
//            t.angular.z = -MAX_ANGULAR_VEL;
            break;
        case(ROTATE_LEFT):
            //rotate to the left direction


            //t.linear.x = smoothUpdateVelocity(t.linear.x,0,0.1);
            t.angular.z = smoothUpdateVelocity(t.angular.z, MAX_ANGULAR_VEL,0.05);
            std::cerr<< "Rotate to Left" <<std::endl;


            t.linear.x = 0;
//            t.angular.z = MAX_ANGULAR_VEL;

            break;
        case(FORWORD):
            //go forword

            t.linear.x = smoothUpdateVelocity(t.linear.x,MAX_LINEAR_VEL,0.05);
            //t.linear.z = smoothUpdateVelocity(t.angular.z,0,0.1);
            std::cerr<< "Move Forward" <<std::endl;


//            t.linear.x = MAX_LINEAR_VEL;
            t.angular.z = 0;

            break;
        case(STOP):
            t.linear.x = 0;
            t.angular.z = 0;
            std::cerr<< "Stop" <<std::endl;
            break;
        }


        pub_twist.publish(t);
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}
