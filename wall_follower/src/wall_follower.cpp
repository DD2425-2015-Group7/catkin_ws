#include <ros/ros.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

double alpha = -0.01;
unsigned int distance_lefts_front = 0;
unsigned int distance_lefts_back = 0;
unsigned int distance_front_left = 0;
unsigned int distance_front_right = 0;
unsigned int distancef_rights_front = 0;
unsigned int distanceb_rights_back = 0;
// Private parameter for the linear speed
float l_speed = 0.2;
float a_speed = 0.2;
// The minimum distance allowed between the front sensors and a wall
float front_limit = 0.1;
float side_limit = 0.1;
float front_limit_max = 0.50;
float side_limit_max = 0.35;

bool side = 0;

enum States{
    ST_empty = 0,
    ST_right_RD,
    ST_left_LD,

    //ST_blocked,

    ST_frontboth,
    ST_front_Left,
    ST_front_Right,

};

void distanceCallback(const ras_arduino_msgs::ADConverter::ConstPtr &msg)
{
  distance_lefts_front = msg->ch1;
  distance_lefts_back = msg->ch2;
  distance_front_left = msg->ch4;
  distance_front_right = msg->ch5;
  distancef_rights_front = msg->ch7;
  distanceb_rights_back = msg->ch8;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_following_controller");

  ros::NodeHandle n;

  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("motor_controller/twist", 1000);

  ros::Subscriber dist_sub = n.subscribe("arduino/adc", 1000, distanceCallback);

  ros::Rate loop_rate(10);

  geometry_msgs::Twist msg;
  States st = ST_empty;

  while (ros::ok()) {
    // Booleans that indicate if the sensors on each side detect something close
    bool frontboth = (distance_front_left < front_limit) && (distance_front_right < front_limit);
    bool front_left = (distance_front_left < front_limit);
    bool front_right = (distance_front_right < front_limit);
    bool right = (distancef_rights_front < side_limit) && (distanceb_rights_back < side_limit);
    bool left = (distance_lefts_front < side_limit) && (distance_lefts_back < side_limit);
    bool front_max = !frontboth && (distance_front_left < front_limit_max) && (distance_front_right < front_limit_max);

    if(frontboth){
        st = ST_frontboth;
    }else if(front_left && !front_right){
        st = ST_front_Left;
    }else if(front_right && !front_left){
        st = ST_front_Right;
    }else if(!frontboth && !front_left && !front_right && !left && !right){ //no one is inside the limit
        st = ST_empty;
    }else if(front_max && (side == 0)){
        if(distancef_rights_front < distance_lefts_front ){
            st = ST_right_RD;
        }else{
            st = ST_left_LD;
            side = 1;
        }
    }else if(front_max && (side == 1)){
         if(distance_lefts_front < distancef_rights_front){
             st = ST_left_LD;
         }else{
             st = ST_right_RD;
             side = 0;
         }
    }



    switch(st) {
        case ST_empty:
            msg.linear.x = l_speed;
            msg.angular.z = 0;
            break;
        case ST_right_RD:
            msg.linear.x = l_speed;
            msg.angular.z = alpha * ( (double)distancef_rights_front- (double)distanceb_rights_back);
            /*ROS_INFO("Linear velocity :%f", msg.linear.x);
                ROS_INFO("Angular velocity :%f", msg.angular.z);*/

            break;
        case  ST_left_LD:
            msg.linear.x = l_speed;
            msg.angular.z = alpha*((double)distance_lefts_front - (double)distance_lefts_back);
            ROS_INFO("Linear velocity: %f", msg.linear.x);
            ROS_INFO("Angular velocity :%f", msg.angular.z);
            break;
        case ST_frontboth:
            msg.linear.x = 0;
            if(side == 0){
                msg.angular.z = -a_speed;

            }else{
                msg.angular.z = a_speed;
            }
            break;
        case ST_front_Left:
            msg.linear.x = 0;
            msg.angular.z = a_speed;
            break;
        case ST_front_Right:
            msg.linear.x = 0;
            msg.angular.z = -a_speed;
            break;
    }


    // P-controller to generate the angular velocity of the robot, to follow the wall
    //msg.angular.z = alpha * ( (double)distance_lefts_front- (double)distance_lefts_back);
    //ROS_INFO("Linear velocity :%f", msg.linear.x);
    //ROS_INFO("Angular velocity :%f", msg.angular.z);
    twist_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0 ;
}


