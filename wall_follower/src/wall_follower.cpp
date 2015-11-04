#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "ir_sensors/RangeArray.h"
#include <cmath>
#include <stdlib.h>
#include <time.h>

double alpha = 25;
// The minimum distance allowed between the front sensors and a wall
float front_limit = 0.25;
float side_limit = 0.15;
float init_front_limit_max = 0.5;
float init_side_limit_max = 0.5;

double distance_lefts_front = init_side_limit_max;
double distance_lefts_back = init_side_limit_max;
double distance_front_left = init_front_limit_max;
double distance_front_right = init_front_limit_max;
double distance_rights_front = init_side_limit_max;
double distance_rights_back = init_side_limit_max;

// Private parameter for the linear speed
float l_speed = 0.2;
float a_speed = 1.5;

// Coresponding to the right decision side
bool side = 0;

bool flagReady = 0;

enum States{
    EMPTY = 0,
    RIGHT,
    LEFT,
    FRONT
};

double smoothUpdateVelocity(double current, double required, double step){
    double diff = current - required;
    if(fabs(diff) > 1.5*step){
       current -= (diff > 0) ? step : -step;
    } else {
        current = required;
    }
    return current;
}

void distanceCallback(const ir_sensors::RangeArray::ConstPtr &msg)
{
    assert(msg->array.size() == 6);
    
    distance_lefts_front = msg->array[0].range;
    distance_lefts_back = msg->array[1].range;
    distance_front_left = msg->array[2].range;
    distance_front_right = msg->array[3].range;
    distance_rights_front = msg->array[4].range;
    distance_rights_back = msg->array[5].range;

    flagReady = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_following_controller");

    ros::NodeHandle n;

    ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Subscriber dist_sub = n.subscribe("/ir_publish/sensors", 1000, distanceCallback);

    ros::Rate loop_rate(25);

    geometry_msgs::Twist msg;

    srand(time(NULL));

    States state = FRONT;
    int counter = 0;
    // Booleans that indicate if the sensors on each side detect something close
    int side=1;

    while (ros::ok()){

        bool front = (distance_front_left < front_limit) || (distance_front_right < front_limit);
        bool right = (distance_rights_front < side_limit) && (distance_rights_back < side_limit);
        bool left = (distance_lefts_front < side_limit) && (distance_lefts_back < side_limit);
        double right_average = (distance_rights_front + distance_rights_back)/2;
        double left_average = (distance_lefts_front + distance_lefts_back)/2;

        if (!flagReady) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // Update of the state
        if(front){ // The front is the most prior
            if(right_average > left_average){
                side = 0;
            }else{
                side = 1;
            }
            state = FRONT;
            std::cerr << "FRONT" << std::endl;
        } else if (right) { // The right is prefered
            /*if(state != RIGHT){
                direction = rand()%2;
                std::cerr << direction << std::endl;
            }*/
            state = RIGHT;
            std::cerr << "RIGHT" << std::endl;
        } else if (left) {
            /*if(state != LEFT){
                direction = rand()%2;
                std::cerr << direction << std::endl;
            }*/
            //direction = rand()%1;

            state = LEFT;
            std::cerr << "LEFT" << std::endl;
        } else {
            state = EMPTY;
            std::cerr << "EMPTY" << std::endl;
        }

        // Action depending on the state
        switch(state) {
        case(EMPTY): // Just go straight
            msg.linear.x = smoothUpdateVelocity(msg.linear.x, l_speed, 0.05);
            msg.angular.z = smoothUpdateVelocity(msg.angular.z, 0, 0.1);
            break;
        case(FRONT):
            msg.linear.x = 0;
            if(side == 1){
                msg.angular.z = smoothUpdateVelocity(msg.angular.z, a_speed, 0.1);
            }else{
                msg.angular.z = smoothUpdateVelocity(msg.angular.z, -a_speed, 0.1);
            }

            break;
        case(RIGHT):
            msg.linear.x = smoothUpdateVelocity(msg.linear.x, l_speed, 0.05);
            msg.angular.z = - alpha * ( (double)distance_rights_front - (double)distance_rights_back);
            break;
        case(LEFT):
            msg.linear.x = smoothUpdateVelocity(msg.linear.x, l_speed, 0.05);
            msg.angular.z = alpha * ( (double)distance_lefts_front - (double)distance_lefts_back);
            break;
        default:
            break;
        }

        // P-controller to generate the angular velocity of the robot, to follow the wall
        //msg.angular.z = alpha * ( (double)distance_lefts_front- (double)distance_lefts_back);
        //ROS_INFO("Linear velocity :%f", msg.linear.x);
        //ROS_INFO("Angular velocity :%f", msg.angular.z);
        if(counter > 10){
            twist_pub.publish(msg);
        }else {
            counter += 1;
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0 ;
}


