#include "logic/FunctionBlocks.h"

FunctionBlocks::FunctionBlocks(ros::NodeHandle& n)
{
    this->timeout = 0;
    srand (time(NULL));
    n.param<int>("map_occupied_min_threshold", this->minOccupied, 10);
    init_mcl_pub = new ros::Publisher();
    *init_mcl_pub = n.advertise<geometry_msgs::Pose>("/mcl/initial_pose", 2, true); //use latch
    
    //Wait 8 s for the map service.
    if(!ros::service::waitForService("/map_node/get_map", 8000)){ 
        ROS_ERROR("Map service unreachable.");
        return;
    }
    mapInflated = new nav_msgs::OccupancyGrid();
    map_client = new ros::ServiceClient();
    *map_client = n.serviceClient<map_tools::GetMap>("/map_node/get_map");
    updateMap();
    
    objectsVision = new classification::ClassifiedObjectArray();
    objectsMap = new classification::ClassifiedObjectArray();
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

bool FunctionBlocks::updateMap(void)
{
    map_tools::GetMap srv2;
    srv2.request.type.data = "inflated";
    if (map_client->call(srv2)){
        *mapInflated = srv2.response.map;
        mapXsz = mapInflated->info.width * mapInflated->info.resolution;
        mapYsz = mapInflated->info.height * mapInflated->info.resolution;
    }else{
        ROS_ERROR("Failed to call service GetMap (inflated).");
        return false;
    }
    return true;
}

int FunctionBlocks::getMapValue(nav_msgs::OccupancyGrid& m, double x, double y)
{
    int xi, yi;
    if(x < 0.0 || y < 0.0)
        return -1;
    xi = (x/m.info.resolution);
    yi = (y/m.info.resolution);
    if(xi >= m.info.width - 1)
        return -1;
    if(yi >= m.info.height - 1)
        return -1;
    return m.data[yi * m.info.width + xi];
}

bool FunctionBlocks::isPointFree(double x, double y)
{
    int v = getMapValue(*mapInflated, x, y);
    if(v < 0)
        return false;
    return (v < minOccupied);
}

geometry_msgs::Pose FunctionBlocks::randUniform(void)
{
    geometry_msgs::Pose rs;
    assert(mapXsz > 0);
    assert(mapYsz > 0);
    double yaw = (((double)rand()/(double)(RAND_MAX/2))-1.0)*M_PI;
    rs.orientation = tf::createQuaternionMsgFromYaw(yaw);
    do{
        rs.position.x = mapXsz * ((double)rand()/(double)(RAND_MAX));
        rs.position.y = mapYsz * ((double)rand()/(double)(RAND_MAX));
    }while(!isPointFree(rs.position.x, rs.position.y));
    return rs;
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
    updateMap();
    geometry_msgs::Pose p, next;
    p = randUniform();
    next = p;
    double dist, maxDist = dist2goal(p);
    for(int i = 1; i < 50; i++){
        p = randUniform();
        dist = dist2goal(p);
        if(dist > maxDist){
            maxDist = dist;
            next = p;
        }
    }
    return next;
}

void FunctionBlocks::testExploration(void)
{
    geometry_msgs::Pose p = exploreNext();
    ROS_INFO("Next to explore: x = %f, y = %f.", p.position.x, p.position.y);
}

geometry_msgs::Pose FunctionBlocks::fetchNext(void)
{
    //TODO: Not tested.
    geometry_msgs::Pose p;
    p.position.x = 0;
    p.position.y = 0;
    p.position.z = 2.0;
    if(objectsMap->objects.size() < 1)
        return p;
    p = objectsMap->objects[0].viewPose;
    double minTime = time2goal(p);
    double t;
    for(int i = 0; i < objectsMap->objects.size(); i++){
        t = time2goal(objectsMap->objects[i].viewPose);
        if(t < minTime){
            minTime = t;
            p = objectsMap->objects[i].viewPose;
        }
    }
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
