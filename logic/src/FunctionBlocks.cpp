#include "logic/FunctionBlocks.h"

FunctionBlocks::FunctionBlocks(ros::NodeHandle& n)
{
    this->timeout = 0;
    init_mcl_pub = new ros::Publisher();
    *init_mcl_pub = n.advertise<geometry_msgs::Pose>("/mcl/initial_pose", 2, true); //use latch
}
    
classification::ClassifiedObjectArray FunctionBlocks::processObject(void)
{
    classification::ClassifiedObjectArray ca;
    return ca;
}

void FunctionBlocks::add2map(classification::ClassifiedObjectArray &)
{
    
}

bool FunctionBlocks::objectDetected(void)
{
    return false;
}


geometry_msgs::Pose FunctionBlocks::randUniform(void)
{
    geometry_msgs::Pose p;
    return p;
}

double FunctionBlocks::dist2goal(geometry_msgs::Pose&)
{
    return 0.6;
}
    
int FunctionBlocks::time2goal(geometry_msgs::Pose&)
{
    return 10;
}

bool FunctionBlocks::poseReached(geometry_msgs::Pose&, double radius, double yaw)
{
    return false;
}

void FunctionBlocks::go2goal(geometry_msgs::Pose&)
{
    
}

void FunctionBlocks::turn(double yaw) 
{
    
}

void FunctionBlocks::setWallFollower(bool on)
{
    
}


geometry_msgs::Pose FunctionBlocks::exploreNext(void)
{
    geometry_msgs::Pose p;
    return p;
}

geometry_msgs::Pose FunctionBlocks::fetchNext(void)
{
    geometry_msgs::Pose p;
    return p;
}


void FunctionBlocks::speak(std::string text)
{
    
}

void FunctionBlocks::sendEvidence(classification::ClassifiedObjectArray &)
{
    
}

void FunctionBlocks::openDoor(void)
{
    this->speak("Starting.");
}

void FunctionBlocks::startTimer(const int seconds)
{
    timeout = seconds;
    time0 = time(NULL);
    assert(time0 != (time_t)(-1));
}

int FunctionBlocks::secondsLeft(void)
{
    int ellapsed = (int) difftime(time(NULL), time0);
    if(ellapsed >= timeout){
        return 0;
    }else{
        return timeout - ellapsed;
    }
}

bool FunctionBlocks::testTimer(void)
{
    startTimer(4);
    assert(secondsLeft()==4);
    sleep(1);
    assert(secondsLeft()==3);
    sleep(3);
    assert(secondsLeft()==0);
    sleep(2);
    assert(secondsLeft()==0);
    startTimer(2);
    assert(secondsLeft()==2);
    sleep(3);
    assert(secondsLeft()==0);
    return true;
}

void FunctionBlocks::initPose(geometry_msgs::Pose& p)
{
    p.position.z = 0.0;
    ros::Rate loop_rate(5);
    loop_rate.sleep();
    init_mcl_pub->publish(p);
}

void FunctionBlocks::initUnknown(void)
{
    geometry_msgs::Pose p;
    p.position.x = 0;
    p.position.y = 0;
    p.position.z = 2.0;
    ros::Rate loop_rate(5);
    loop_rate.sleep();
    init_mcl_pub->publish(p);
}

void FunctionBlocks::testMclInit(void)
{
    geometry_msgs::Pose pose;
    pose.position.x = 0.13;
    pose.position.y = 0.2;
    pose.orientation = tf::createQuaternionMsgFromYaw(0);
    initPose(pose);
    sleep(2);
    initUnknown();
    sleep(2);
    initPose(pose);
    sleep(2);
    initPose(pose);
}

bool FunctionBlocks::isLocalized(void)
{
    return true;
}

void FunctionBlocks::objects2localize(classification::ClassifiedObjectArray &)
{
    
}

int FunctionBlocks::objectMapped(classification::ClassifiedObject &)
{
    return -1; //Not mapped.
}
