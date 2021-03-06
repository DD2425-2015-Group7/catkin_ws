#ifndef _FUNCTION_BLOCKS_H
#define _FUNCTION_BLOCKS_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
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
#include "map_tools/ObjectStorage.h"

#include "path_planning/GetPath.h" 

#include "motion_controllers/GetPathPoints.h"

#include "classification/ClassifiedObject.h"
#include "classification/ClassifiedObjectArray.h"

#include <vector>
#include <cmath>
#include <time.h>
#include <random>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <unordered_map>

#include "ras_msgs/RAS_Evidence.h"

class FunctionBlocks
{
    public:
        FunctionBlocks(ros::NodeHandle& n);
    
        //Vision and mapping.
        classification::ClassifiedObjectArray processObject(void);
	classification::ClassifiedObjectArray processObjectSophisticated(void);
        void add2map(classification::ClassifiedObjectArray &);  //Includes cam_link -> map tf.
        bool sendObjects(classification::ClassifiedObjectArray& objects);
        void setViewPose(classification::ClassifiedObject& obj);
        void testAdd2Map(void);
        bool objectDetected(void);
        bool loadObjects(std::string bagFile);
        bool saveObjects(std::string bagFile);
     
        //Motion.
	nav_msgs::Path getPath(geometry_msgs::Pose&);
        double dist2goal(geometry_msgs::Pose&);
        int time2goal(geometry_msgs::Pose&); 
        bool poseReached(geometry_msgs::Pose&, double radius, double yaw);
        void go2goal(geometry_msgs::Pose&);
        void turn(double yaw);
        void setWallFollower(bool on);
        void stopRobotAStar(void);
        void testPathPlanning(void);

        //Decisions.
        bool isPointFree(double x, double y);
        geometry_msgs::Pose randUniform(void);
        geometry_msgs::Pose exploreNext(void);
        void testExploration(void);
        geometry_msgs::Pose fetchNext(void);
        
        //User interface.
        void publishing(void);
        void testReporting(void);
        void speak(std::string text);
        void sendEvidence(classification::ClassifiedObjectArray &);
        void reportState(std::string text, int verbose);
        void openDoor(void);
        void startTimer(const int seconds);
        int secondsLeft(void);
        bool testTimer(void);
        
        //Localization.
        void initPose(geometry_msgs::Pose&);
        void initUnknown(void);
        void testMclInit(void);
        void mclCB(const std_msgs::Bool::ConstPtr& msg);
        bool isLocalized(void);
        void objects2localize(classification::ClassifiedObjectArray &);
        
	void visionCB(const classification::ClassifiedObjectArray::ConstPtr &);
	void odomCB(const nav_msgs::Odometry::ConstPtr &);

    private:
        time_t time0;
        int timeout;
        
        double mapXsz, mapYsz;
        int minOccupied;
        bool mclReady, mclLocalized;
        
        nav_msgs::OccupancyGrid *mapInflated;
        ros::ServiceClient *getPath_client, *getPathPoints_client;
        ros::ServiceClient *map_client, *add_objects_client, *object_storage_client;
        ros::Publisher *init_mcl_pub;
        
        nav_msgs::Path *currentPath;
        ros::Publisher *state_marker_pub, *path_pub;
        visualization_msgs::MarkerArray *all_markers;
        
	ros::Publisher *espeak_pub;
	ros::Subscriber *vision_sub;
	ros::Subscriber *odom_sub;
    ros::Subscriber *mcl_sub;
	ros::Publisher *wallfol_pub;
	ros::Publisher *evidence_pub;
	ros::Publisher *twist_pub;
	ros::Publisher *goalPose_pub;
    
        bool justStarted;
        classification::ClassifiedObjectArray *objectsVision, *objectsMap, *objects2visit;
        
        bool updateMap(void);
        int getMapValue(nav_msgs::OccupancyGrid& m, double x, double y);
	//int objectMapped(classification::ClassifiedObject &);
        
	std::string MapFrameName, RobotFrameName;
	
	tf::TransformListener *tf_listener;

	int countObjDetected; 

	bool odomReady;

	double smoothUpdateVelocity(double current, double required, double step);

	geometry_msgs::Pose odomPose;

	int objDetectTimeout;

};

#endif
