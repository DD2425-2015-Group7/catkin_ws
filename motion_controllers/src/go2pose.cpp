#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "ir_sensors/RangeArray.h"
#include "geometry_msgs/Twist.h"
#include "math.h"


double tolDistL = 0.03, tolDistH = 0.05, tolYawL = 0.2, tolYawH = 0.4;
double maxLinSpeed = 0.2, maxOnSpot = 2.0, minOnSpot = 1.0;
double maxLinAcc = 0.4, maxAngAcc = 5.0;
double p_angle_lin = 0.5, p_angle_spot = 1.5;
double irFrontMax = 0.8, irFrontMin = 0.1;
double irSideMax = 0.2, irSideMin = 0.04, irSideMaxDiff = 0.05;
double robotSafetyRadius = 0.15;
bool wallFollowingEnabled = true;
double p_wall_alpha = 25;

double x, y, yawFin;
double distance, angle;
geometry_msgs::Twist *twist;
bool poseReceived = false;

bool irReceived = false;
double distance_lefts_front;
double distance_lefts_back;
double distance_front_left;
double distance_front_right;
double distance_rights_front;
double distance_rights_back;

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

void irCB(const ir_sensors::RangeArray::ConstPtr &msg)
{
    assert(msg->array.size() == 6);
    
    distance_lefts_front = msg->array[0].range;
    distance_lefts_back = msg->array[1].range;
    distance_front_left = msg->array[2].range;
    distance_front_right = msg->array[3].range;
    distance_rights_front = msg->array[4].range;
    distance_rights_back = msg->array[5].range;

    irReceived = true;
}

int wallAvailable(void)
{
    // Choose which wall to follow.
    // 0 none, -1 left, 1 right
    if(!wallFollowingEnabled)
        return 0;
        
    if(!irReceived)
        return 0;
    
    if(distance_rights_front > irSideMin && distance_rights_front < irSideMax
        && distance_rights_back > irSideMin && distance_rights_back < irSideMax){
            if(abs(distance_rights_back - distance_rights_front) < irSideMaxDiff){
                return 1;
            }
    }
    
    if(distance_lefts_front > irSideMin && distance_lefts_front < irSideMax
        && distance_lefts_back > irSideMin && distance_lefts_back < irSideMax){
            if(abs(distance_lefts_back - distance_lefts_front) < irSideMaxDiff){
                return -1;
            }
    }
    
    return 0;
}

double followWall(int wall)
{
    if(!irReceived)
        return 0;
    if(!wallFollowingEnabled || wall == 0)
        return 0;

    if(wall>0)
        return -p_wall_alpha * ( (double)distance_rights_front - (double)distance_rights_back);
    else
        return p_wall_alpha * ( (double)distance_lefts_front - (double)distance_lefts_back);
}

double frontWallDist(void)
{
    if(!irReceived)
        return INFINITY;
        
    if(distance_front_left > irFrontMax || distance_front_right > irFrontMax)
        return INFINITY;
    if(distance_front_left < irFrontMin || distance_front_right < irFrontMin)
        return INFINITY;
        
    if(distance_front_left < distance_front_right)
        return distance_front_left - robotSafetyRadius;
    else
        return distance_front_right - robotSafetyRadius;
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
    bool wallFollowing;
    if(distance > (0.5 * vx * vx)/maxLinAcc &&
            frontWallDist() > (0.5 * vx * vx)/maxLinAcc){
        vx += maxLinAcc * dt;
        wallFollowing = true;
    }else{
        vx -= maxLinAcc * dt;
        wallFollowing = false;
    }
    if(vx > maxLinSpeed){
        vx = maxLinSpeed;
    }
    if(vx < 0.0){
        vx = 0.0;
    }
    twist->linear.x = vx;
    int wall = wallAvailable();
    if(wall && wallFollowing){
        twist->angular.z = followWall(wall);
    }else{
        twist->angular.z = p_angle_lin * angle;
    }
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
    
    ros::Subscriber dist_sub = n.subscribe("/ir_publish/sensors", 5, irCB);
    
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
    n.param<bool>("go2pose_wall_following_enabled", wallFollowingEnabled, false);
    n.param<double>("p_wall_alpha", p_wall_alpha, 25.0);
    n.param<double>("robot_safety_radius", robotSafetyRadius, 0.15);
    
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
        
        //TODO: test if wall checking prevents reaching the desired goal.
        // If that is the case, integrate pub_points_list and go2pose into one node.
        if(distance > tolDistL && distance < frontWallDist() && !turningEnabled){
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
