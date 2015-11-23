#ifndef _FUNCTION_BLOCKS_H
#define _FUNCTION_BLOCKS_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

#include "map_tools/AddEllipse.h"
#include "map_tools/GetMap.h"

#include "classification/ClassifiedObject.h"
#include "classification/ClassifiedObjectArray.h"

#include <vector>
#include <cmath>

class FunctionBlocks
{
    public:
        FunctionBlocks(void);
    
        //Vision and mapping.
        classification::ClassifiedObjectArray processObject(void); //Blocking function.
        void add2map(classification::ClassifiedObjectArray &);  //Includes cam_link -> map tf.
        bool objectDetected(void);
        
        //Motion.
        geometry_msgs::Pose randUniform(void);
        double dist2goal(geometry_msgs::Pose&);
        int time2goal(geometry_msgs::Pose&);
        bool poseReached(geometry_msgs::Pose&, double radius, double yaw);
        void go2goal(geometry_msgs::Pose&);
        void turn(double yaw); //Blocking function.
        void setWallFollower(bool on);
        
        //Decisions.
        geometry_msgs::Pose exploreNext(void);
        geometry_msgs::Pose fetchNext(void);
        
        //User interface.
        void speak(std::string text);
        void sendEvidence(classification::ClassifiedObjectArray &);
        void openDoor(void);
        void startTimer(const int seconds);
        int secondsLeft(void);
        
        //Localization.
        void initPose(geometry_msgs::Pose&);
        void initUnknown(void);
        bool isLocalized(void);
        void objects2localize(classification::ClassifiedObjectArray &);
        
    private:
        classification::ClassifiedObjectArray *objectsVision, *objectsMap;
        
        int objectMapped(classification::ClassifiedObject &); //return index
        
};

#endif
