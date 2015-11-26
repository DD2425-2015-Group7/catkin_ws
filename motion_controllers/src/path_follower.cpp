#include "ros/ros.h"
#include "ir_sensors/RangeArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "tf/LinearMath/Transform.h"
#include "tf/transform_datatypes.h"
#include <ras_arduino_msgs/ADConverter.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>

double distance;
double safetyDistance = 0;
double angle;

double safetyAngle = 0.4;

double MAX_LINEAR_VEL = 0.2;
double MAX_ANGULAR_VEL = 1.5;


double alpha = 25;
// The minimum distance allowed between the front sensors and a wall
float front_limit = 0.25;
float side_limit = 0.15;
float init_front_limit_max = 0.5;
float init_side_limit_max = 0.5;

double distance_rights_front = init_side_limit_max;
double distance_rights_back = init_side_limit_max;
double distance_lefts_front = init_side_limit_max;
double distance_lefts_back = init_side_limit_max;
//double distance_front_left = init_front_limit_max;
//double distance_front_right = init_front_limit_max;

ros::Publisher *pub_twist;

enum States
{
    STOP,
    ROTATE_RIGHT,
    ROTATE_LEFT,
    FOLLOW_RIGHT_WALL,
    FOLLOW_LEFT_WALL,
    //FOLLOW_WALL,
    FORWORD
};

States state;

//No need to care about the front sensors, right?
void distanceCallback(const ir_sensors::RangeArray::ConstPtr &msg)
{
    // what's this one
      //assert(msg->array.size() == 6);

    distance_lefts_front = msg->array[0].range;
    distance_lefts_back = msg->array[1].range;
    distance_rights_front = msg->array[4].range;
    distance_rights_back = msg->array[5].range;

}


void kobukiDistanceCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
{
    /*
     * Information about the Kobuki IR sensors
     *
        Frame: distance_sensor_front_left_link; reported on adc channel 1.
        Frame: distance_sensor_back_left_link; reported on adc channel 2.
        Frame: distance_sensor_front_right_link; reported on adc channel 3.
        Frame: distance_sensor_back_right_link; reported on adc channel 4.
        Frame: distance_sensor_forward_right_link; reported on adc channel 5.
        Frame: distance_sensor_forward_left_link; reported on adc channel 6.
    */

   // Kobuki ADC setting
        distance_lefts_front = msg->ch1 ;
        std::cout<< "Left front sensors"<< distance_lefts_front<<std::endl;
        distance_lefts_back = msg->ch2;
        std::cout<< "Left back sensors"<< distance_lefts_back<<std::endl;
        distance_rights_front = msg->ch3;
        std::cout<< "Right front sensors"<< distance_rights_back<<std::endl;
        distance_rights_back = msg->ch4;
        std::cout<< "Right back sensors"<< distance_rights_back<<std::endl;
}


void setPosition(const geometry_msgs::Pose::ConstPtr& msg)
{
    //As we use the tf package to tansform the point, so the distance will be just the x value.
    distance = msg.get()->position.x;

    if(msg.get()->position.y > 0.001 ||  msg.get()->position.y < -0.001)
    {
        angle = atan2(msg.get()->position.y, msg.get()->position.x);
    }
    else
    {
        angle = 0.0;
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
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle handle;
    ros::Subscriber sub_posi = handle.subscribe("/path_pose", 1000, setPosition);

    ros::Subscriber sub_IRdist = handle.subscribe("/ir_publish/sensors", 1000, distanceCallback);
    ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    // These two lines are for kobuki simulationd
    //ros::Subscriber sub_IRdist = handle.subscribe("/kobuki/adc", 1000, kobukiDistanceCallback);
    //ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);
    geometry_msgs::Twist t;
    ros::Rate loopRate(10);

    while(ros::ok())
    {
        bool right = (distance_rights_front < side_limit) && (distance_rights_back < side_limit);
        bool left = (distance_lefts_front < side_limit) && (distance_lefts_back < side_limit);

        //Do we need to say we need to follow the right wall or the left wall? Right now, I would say "No"
        ROS_INFO("Angle is :%f",angle);
        if(angle > safetyAngle)
        {
            ROS_INFO("Change state to Left!!!!");
            state = ROTATE_LEFT;
        }
        else if(angle < -safetyAngle)
        {
            ROS_INFO("change state to Right!!!!");
            state = ROTATE_RIGHT;
        }
        else if (distance > safetyDistance)
        {
            if(right)
            {
                ROS_INFO("Follow the right wall!!!!");
                state = FOLLOW_RIGHT_WALL;
            }
            else if(left)
            {
                ROS_INFO("Follow the left wall!!!!");
                state = FOLLOW_LEFT_WALL;
            }
            else
            {
                ROS_INFO("just move forward!!!!");
                state = FORWORD;
            }
        }
        else
        {
            ROS_INFO("STOP!!!!!!");
            state = STOP;
        }

        switch(state)
        {
        case(ROTATE_RIGHT):
            //rotate to the right direction

            //t.linear.x = smoothUpdateVelocity(t.linear.x,0,0.1);
            //t.angular.z = -MAX_ANGULAR_VEL;
            t.linear.x = 0;
            //t.angular.z = smoothUpdateVelocity(t.angular.z, -MAX_ANGULAR_VEL,0.025);
            std::cerr<< "Rotate to Right" <<std::endl;

            t.angular.z = 3 * angle;

            if(t.angular.z < -MAX_ANGULAR_VEL)
            {
               t.angular.z = -MAX_ANGULAR_VEL;
            }

            break;
        case(ROTATE_LEFT):
            //t.angular.z = MAX_ANGULAR_VEL;
            //t.linear.x = smoothUpdateVelocity(t.linear.x,0,0.1);
            t.angular.z = 3 * angle;

            if(t.angular.z > MAX_ANGULAR_VEL)
            {
                t.angular.z = MAX_ANGULAR_VEL;
            }

            t.linear.x = 0;
            //t.angular.z = smoothUpdateVelocity(t.angular.z, MAX_ANGULAR_VEL,0.025);
            std::cerr<< "Rotate to Left" <<std::endl;
            break;

        case(FOLLOW_LEFT_WALL):
            t.linear.x = smoothUpdateVelocity(t.linear.x, MAX_LINEAR_VEL, 0.05);
            t.angular.z = alpha * ( (double)distance_lefts_front - (double)distance_lefts_back);
            break;
        case(FOLLOW_RIGHT_WALL):
            t.linear.x = smoothUpdateVelocity(t.linear.x, MAX_LINEAR_VEL, 0.05);
            t.angular.z = - alpha * ( (double)distance_rights_front - (double)distance_rights_back);
            break;

        case(FORWORD):
            //t.linear.x = MAX_LINEAR_VEL;
            //t.linear.z = smoothUpdateVelocity(t.angular.z,0,0.1);
             t.angular.z = 2 * angle;

            //t.angular.z = 0;
            t.linear.x = smoothUpdateVelocity(t.linear.x,MAX_LINEAR_VEL,0.05);
            std::cerr<< "Move Forward" <<std::endl;
            break;
        case(STOP):
            //stop
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
