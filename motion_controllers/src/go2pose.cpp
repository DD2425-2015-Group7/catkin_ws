#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/Twist.h"
#include "math.h"


double tolDistL = 0.03, tolDistH = 0.05, tolYawL = 0.2, tolYawH = 0.4;
double maxLinSpeed = 0.2, maxOnSpot = 2.0, minOnSpot = 1.0;
double maxLinAcc = 0.4, maxAngAcc = 5.0;
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
        vx -= maxLinAcc * dt;
    }
    if(vx > maxLinSpeed){
        vx = maxLinSpeed;
    }
    if(vx < 0.0){
        vx = 0.0;
    }
    twist->linear.x = vx;
    twist->angular.z = p_angle_lin * angle;
    twist->angular.z = boundaries(twist->angular.z, 0.0, maxOnSpot);
}

void turning(double a, double dt)
{
    double az = twist->angular.z;
    twist->linear.x = 0.0;
    //twist->angular.z = p_angle_spot * a;
    if(a > (0.5 * az * az)/maxAngAcc ){
        az += maxAngAcc * dt;
    }else{
        az -= maxAngAcc * dt;
    }
    twist->angular.z = az;
    twist->angular.z = boundaries(twist->angular.z, 0.0, maxOnSpot);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "go2pose");
    ros::NodeHandle n("/go2pose");

    ros::Subscriber sub_posi = n.subscribe("/path_pose", 2, setPose);
    // "/mobile_base/commands/velocity" "/cmd_vel"
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel",2); 
    twist = new geometry_msgs::Twist();
    
    n.param<double>("p_angle_lin", p_angle_lin, 2.0);
    n.param<double>("p_angle_spot", p_angle_spot, 1.5);
    n.param<double>("yaw_tolerance_low", tolYawL, 0.2);
    n.param<double>("yaw_tolerance_high", tolYawH, 0.3);
    n.param<double>("distance_tolerance_low", tolDistL, 0.03);
    n.param<double>("distance_tolerance_high", tolDistH, 0.05);
    n.param<double>("max_lin_speed", maxLinSpeed, 0.3);
    n.param<double>("max_on_spot_angular_speed", maxOnSpot, 3.0);
    n.param<double>("min_on_spot_angular_speed", minOnSpot, 1.0);
    n.param<double>("max_lin_acc", maxLinAcc, 0.5);
    n.param<double>("max_ang_acc", maxAngAcc, 5.0);
    
    double rate = 20;
    int count = 0;
    double dt = 1.0/rate;
    bool turningEnabled = false;
    enum State{t1, fw, t2, stop};
    enum State curs, prevs;
    prevs = fw;
    curs = stop;
    yawFin = distance = angle = 0.0;
    
    ros::Rate loopRate(rate);

    while(ros::ok())
    {
        if(fabs(angle) < tolYawL)
            turningEnabled = false;
        if(fabs(angle) > tolYawH)
            turningEnabled = true;
        
        if(distance > tolDistL && !turningEnabled){
            curs = fw;
            goForward(dt);
        }else if(distance > tolDistH && turningEnabled){
            curs = t1;
            turning(angle, dt);
        }else if(fabs(yawFin) > tolYawL && distance < tolDistH){
            curs = t2;
            turning(yawFin, dt);
        }else{
            curs = stop;
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
        }
        
        //Stop on no pose received.
        if(poseReceived){
            count = 0;
            poseReceived = false;
        }else if(count > (int)rate){
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
            curs = stop;
        }else{
            count++;
        }
        
        //Reset pwm_controller on state change.
        if(prevs != curs){
            twist->linear.x = 0.0;
            twist->angular.z = 0.0;
        }
        
        //Publish only when not completely stopped.
        if(prevs==stop && curs==stop){
            
        }else{
            pub_twist.publish(*twist);
        }
        
        prevs = curs;
        ros::spinOnce();
        loopRate.sleep();
    }

   return 0;
}
