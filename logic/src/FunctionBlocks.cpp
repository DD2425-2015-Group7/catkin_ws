#include "logic/FunctionBlocks.h"

FunctionBlocks::FunctionBlocks(void)
{
    
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
    
}

void FunctionBlocks::startTimer(const int seconds)
{
    
}

int FunctionBlocks::secondsLeft(void)
{
    return 100;
}

void FunctionBlocks::initPose(geometry_msgs::Pose&)
{
    
}

void FunctionBlocks::initUnknown(void)
{
    
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
