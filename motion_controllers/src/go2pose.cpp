#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/Twist.h"
#include "math.h"


double tolDist = 0.03, tolYaw = 0.2;
double maxLinAcc = 0.4, maxLinSpeed = 0.2, maxOnSpot = 2.0, minOnSpot = 1.0;
double p_angle_lin = 0.5, p_angle_spot = 1.5;

double x, y, yawFin;
double distance, angle;
geometry_msgs::Twist *twist;
bool poseReceived = false;



void updateValues(double _x, double _y, double _yawFin)
{
    x = _x;
    y = _y;
    yawFin = _yawFin;
    distance = sqrt(x * x + y * y);
    
    if(y > 0.001 ||  y < -0.001){
        angle = atan2(y, x) ;
    }else{
        angle = 0.0;
    }
    
    if(distance < tolDist)
        angle = 0.0;
} 

void setPose(const geometry_msgs::Pose::ConstPtr& msg)
{
    //Pose is expected to be in the /base_link frame.
    updateValues(msg->position.x, msg->position.y, tf::getYaw(msg->orientation));
    poseReceived = true;
}

double boundaries(double val, double low, double high)
{
    assert(high > low);
    assert(low >= 0.0);
    if(val > high){
        val = high;
    }else if(val < -high){
        val = -high;
    }
    if(val < low && val > -low){
        if(val > 0)
            val = low;
        else
            val = -low;
    }
    return val;
}

void goForward(double dt)
{
    double vx = twist->linear.x;
    if(distance > (0.5 * vx * vx)/maxLinAcc ){
        vx += maxLinAcc * dt;
    }else{
        vx -= maxLinAcc;
    }
    if(vx > maxLinSpeed){
        vx = maxLinSpeed;
    }
    if(vx < 0.0){
        vx = 0.0;
    }
    twist->linear.x = vx;
    twist->angular.z = -p_angle_lin * angle;
    twist->angular.z = boundaries(twist->angular.z, 0.0, maxOnSpot);
}

void turning(double a)
{
    twist->linear.x = 0.0;
    twist->angular.z = -p_angle_spot * a;
    twist->angular.z = boundaries(twist->angular.z, minOnSpot, maxOnSpot);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "go2pose");
    ros::NodeHandle n("/go2pose");

    ros::Subscriber sub_posi = n.subscribe("/path_pose", 2, setPose);
    
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel",2);
    twist = new geometry_msgs::Twist();
    
    n.param<double>("p_angle_lin", p_angle_lin, 0.5);
    n.param<double>("p_angle_spot", p_angle_spot, 1.5);
    n.param<double>("yaw_tolerance", tolYaw, 0.3);
    n.param<double>("distance_tolerance", tolDist, 0.03);
    n.param<double>("max_lin_speed", maxLinSpeed, 0.2);
    n.param<double>("max_on_spot_angular_speed", maxOnSpot, 2.0);
    n.param<double>("min_on_spot_angular_speed", minOnSpot, 1.0);
    n.param<double>("max_lin_acc", maxLinAcc, 0.4);
    
    double rate = 10;
    int count = 0;
    double dt = 1.0/rate;
    bool isForward = false, isStopped = false;
    enum State{t1, fw, t2, stop};
    enum State s;
    s = t1;
    yawFin = distance = angle = 0.0;
    
    ros::Rate loopRate(rate);

    while(ros::ok())
    {
        if(fabs(angle) > 2*tolYaw)
            s = t1;
        if(s == t1){
            isForward = false;
            turning(angle);
            ROS_INFO("turn 1");
            if(fabs(angle) < tolYaw)
                s = fw;
        }else if(s == fw){
            if(!isForward){
                isForward = true;
                isStopped = true;
            }
            ROS_INFO("forward");
            goForward(dt);
            if(distance < tolDist)
                s = t2;
        }else if(s == t2){
            isForward = false;
            turning(yawFin);
            ROS_INFO("turn 2");
            if(fabs(angle) < tolYaw)
                s = stop;
        }else if(s == stop){
            isForward = false;
            ROS_INFO("stop");
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
            s = t1;
        }
        
        if(poseReceived){
            count = 0;
            poseReceived = false;
        }else if(count > (int)rate){
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
        }else{
            count++;
        }
        
        if(isStopped){
            ROS_INFO("isStopped");
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
            isStopped = false;
        }
        
        pub_twist.publish(*twist);
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}
