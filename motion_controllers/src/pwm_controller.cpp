#include "ros/ros.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <mutex>

#define PI_F 3.14159f

std::mutex mtx;
double refOmegaL = 0.0, refOmegaR = 0.0;
int encL = 0, encR = 0;
bool encodersUpdated = false;

// Usable linear velocity range: 0.2 to 0.8 m/s
// Usable angular velocity range, on the spot: 1.0 to 6.0 rad/s

/* 10 rotations
left 6829 -> 10429
right 8493 -> 12091
*/

#define MAX_REF_ERROR 400
#define MIN_PWM -250
#define MAX_PWM 250
#define MAX_STOP_PWM 10
#define MHYST_R (40-MAX_STOP_PWM)
#define MHYST_L (40-MAX_STOP_PWM)

// Hystheresis: + 38 PWM right cca
// Forward direction: 
// 	PWM1 positive, motor A, left wheel, encoder 2(A) increasing
//  PWM2 negative, motor B, right wheel, encoder 1(B) increasing
const double wheelDist = 0.215, wheelRadius = 0.099/2.0; 
const int ticksPerRevolution = 360; 
const int controlRate = 50;
double controlPL = 0.14, controlIL = 0.001, controlDL = 0.0, controlI2L = 0.0;
double controlPR = 0.14, controlIR = 0.001, controlDR = 0.0, controlI2R = 0.0;
int stopped = 1;

void updateTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	mtx.lock();
    if(msg->linear.x > -0.02 && msg->linear.x < 0.02 &&
            msg->angular.z > -0.01 && msg->angular.z < 0.01){
        stopped = 1;
    }else if(stopped != 0){
        stopped = 0;
    }
	refOmegaL = (msg->linear.x - (wheelDist/2.0)*msg->angular.z)/wheelRadius;
	refOmegaR = (msg->linear.x + (wheelDist/2.0)*msg->angular.z)/wheelRadius;
	mtx.unlock();
}

void updateEncoders(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
	mtx.lock();
	encR += msg->delta_encoder1;
	encL += msg->delta_encoder2;
    encodersUpdated = true;
	mtx.unlock();
}

ras_arduino_msgs::PWM motorControl(void)
{
	static int refTicksL = 0, refTicksR = 0;
	static int pwmL = 0, pwmR = 0;
	static int errL = 0, errR = 0, errIntL = 0, errIntR = 0;
    static int errPrevL = 0, errPrevR = 0;
	static int errInt2L = 0, errInt2R = 0;
	static int flagIntL = 1, flagIntR = 1;
    static int flagIntRef = 1;
    static int encCounter = 0;
	
	mtx.lock();
    if(flagIntRef){
        refTicksL += (int)((refOmegaL/(2*PI_F*controlRate))*ticksPerRevolution);
        refTicksR += (int)((refOmegaR/(2*PI_F*controlRate))*ticksPerRevolution);
    }
    if(stopped){
        refTicksL = encL;
        refTicksR = encR;
        errIntL = 0.0;
        errIntR = 0.0;
        errInt2L = 0.0;
        errInt2R = 0.0;
    }
	errL = refTicksL - encL;
	errR = refTicksR - encR;
    if(errL > MAX_REF_ERROR || errL < -MAX_REF_ERROR || errR > MAX_REF_ERROR || errR < -MAX_REF_ERROR){
        flagIntRef = 0;
    }else if(flagIntRef != 1){
        flagIntRef = 1;
    }
	ROS_DEBUG("errL: %d\nerrR: %d\n", errL, errR);
	mtx.unlock();
	if(flagIntL){
		errIntL += errL;
		errInt2L += errIntL;
	}
	if(flagIntR){
		errIntR += errR;
		errInt2R += errIntR;
	}
		
    pwmL = int(controlPL*errL + controlIL*errIntL + controlDL*(errL - errPrevL) + controlI2L*errInt2L);
    pwmR = int(controlPR*errR + controlIR*errIntR + controlDR*(errR - errPrevR) + controlI2R*errInt2R);
    

    errPrevL = errL;
    errPrevR = errR;
	
	/*
	if(pwmL > MAX_STOP_PWM)
		pwmL += MHYST_L;
	else if(pwmL < -MAX_STOP_PWM)
		pwmL -= MHYST_L;
	
	if(pwmR > MAX_STOP_PWM)
		pwmR += MHYST_R;
	else if(pwmR < -MAX_STOP_PWM)
		pwmR -= MHYST_R;
	*/
	
	if(pwmL > MAX_PWM){
		pwmL = MAX_PWM;
		flagIntL = 0;
	}else if(pwmL < MIN_PWM){
		pwmL = MIN_PWM;
		flagIntL = 0;
	}else if(flagIntL != 1){
		flagIntL = 1;
	}
	if(pwmR > MAX_PWM){
		pwmR = MAX_PWM;
		flagIntR = 0;
	}else if(pwmR < MIN_PWM){
		pwmR = MIN_PWM;
		flagIntR = 0;
	}else if(flagIntR != 1){
		flagIntR = 1;
	}
    
    if(abs(pwmL) > 2 || abs(pwmR) > 2){
        if(!encodersUpdated && encCounter < 3*controlRate){
            encCounter++;
        }else{
            encCounter = 0;
            encodersUpdated = false;
        }
            
        if(encCounter > controlRate/5){
            pwmL = 0;
            pwmR = 0;
            ras_arduino_msgs::PWM msg;
            msg.PWM1 = pwmL;
            msg.PWM2 = -pwmR; 
            ROS_WARN("No encoder data for more than 200 ms!");
            return msg;
        }
    }else{
        encCounter = 0;
    } 
	
	ras_arduino_msgs::PWM msg;
	msg.PWM1 = pwmL;
	msg.PWM2 = -pwmR; 
	return msg;
}
  
int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "pwm_controller");
	ros::NodeHandle n("/pwm_controller");;
	
    n.param("right_p", controlPR, 0.7);
    n.param("left_p", controlPL, 0.7);
    n.param("right_i", controlIR, 0.001);
    n.param("left_i", controlIL, 0.001);
    n.param("right_d", controlDR, 0.0);
    n.param("left_d", controlDL, 0.0);
	
	ros::Publisher pwm_pub = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", 1000);
	ros::Subscriber twist_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, updateTwist);
	ros::Subscriber encoder_sub = n.subscribe<ras_arduino_msgs::Encoders>("/arduino/encoders", 1000, updateEncoders);
	ros::Rate loop_rate(controlRate);

	while (ros::ok())
    {

        pwm_pub.publish(motorControl()); //
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
