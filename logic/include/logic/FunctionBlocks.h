#ifndef _FUNCTION_BLOCKS_H
#define _FUNCTION_BLOCKS_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/tf.h"
#include "math.h"

#include "map_tools/AddEllipse.h"
#include "map_tools/AddObjects.h"
#include "map_tools/GetMap.h"

#include "path_planning/GetPath.h" 

#include "classification/ClassifiedObject.h"
#include "classification/ClassifiedObjectArray.h"

#include <vector>
#include <cmath>
#include <time.h>
#include <random>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <unordered_map>

#include "ras_msgs/RAS_Evidence.h"

class FunctionBlocks
{
    public:
        FunctionBlocks(ros::NodeHandle& n);
    
        //Vision and mapping.
        classification::ClassifiedObjectArray processObject(void);
        void add2map(classification::ClassifiedObjectArray &);  //Includes cam_link -> map tf.
        bool sendObjects(classification::ClassifiedObjectArray& objects);
        void setViewPose(classification::ClassifiedObject& obj);
        void testAdd2Map(void);
        bool objectDetected(void);
        
        //Motion.
        double dist2goal(geometry_msgs::Pose&);
        int time2goal(geometry_msgs::Pose&); 
        bool poseReached(geometry_msgs::Pose&, double radius, double yaw);
        void go2goal(geometry_msgs::Pose&);
        void turn(double yaw);
        void setWallFollower(bool on);
        
        //Decisions.
        bool isPointFree(double x, double y);
        geometry_msgs::Pose randUniform(void);
        geometry_msgs::Pose exploreNext(void);
        void testExploration(void);
        geometry_msgs::Pose fetchNext(void);
        
        //User interface.
        void speak(std::string text);
        void sendEvidence(classification::ClassifiedObjectArray &);
        void openDoor(void);
        void startTimer(const int seconds);
        int secondsLeft(void); //Ondrej
        bool testTimer(void);
        
        //Localization.
        void initPose(geometry_msgs::Pose&);
        void initUnknown(void);
        void testMclInit(void);
        bool isLocalized(void);
        void objects2localize(classification::ClassifiedObjectArray &);
        
	void visionCB(const classification::ClassifiedObjectArray::ConstPtr &);
	void odomCB(const nav_msgs::Odometry::ConstPtr &);

    private:
        time_t time0;
        int timeout;
        
        double mapXsz, mapYsz;
        int minOccupied;
        
        nav_msgs::OccupancyGrid *mapInflated;
        ros::ServiceClient *map_client, *add_objects_client, *getPath_client;
        ros::Publisher *init_mcl_pub;
	ros::Publisher *espeak_pub;
	ros::Subscriber *vision_sub;
	ros::Subscriber *odom_sub;
	ros::Publisher *wallfol_pub;
	ros::Publisher *evidence_pub;
	ros::Publisher *twist_pub;
	ros::Publisher *goalPose_pub;

        classification::ClassifiedObjectArray *objectsVision, *objectsMap;
        
        bool updateMap(void);
        int getMapValue(nav_msgs::OccupancyGrid& m, double x, double y);
	//int objectMapped(classification::ClassifiedObject &);
        
	std::string MapFrameName, RobotFrameName;
	
	tf::TransformListener *tf_listener;

	int countObjDetected; 

	bool odomReady;

	double smoothUpdateVelocity(double current, double required, double step);

	geometry_msgs::Pose odomPose;

};

#endif
