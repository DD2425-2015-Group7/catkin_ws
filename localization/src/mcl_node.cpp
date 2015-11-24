#include "math.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "ir_sensors/RangeArray.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "map_tools/GetMap.h"
#include "localization/MonteCarlo.h"
#include "localization/RangeModel.h"

#include <mutex>

#define PI 3.14159

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, ir_sensors::RangeArray> SyncPolicy;

std::string odomFrame, baseFrame, mapFrame;
double irSigma;
double initConeRadius, initYawVar;
int minOccupied;
bool mclEnabled = false;

std::mutex mtx;
struct Poses{
    double x0 = 0.0, y0 = 0.0;
    double x = 0.0, y = 0.0, th = 0.0;
};
struct Poses coords;
ros::Time current_time;

tf::TransformBroadcaster *tf_broadcaster;
tf::TransformListener *tf_listener;
ros::ServiceClient *map_client;

nav_msgs::OccupancyGrid *mapInflated, *mapDistance;
MonteCarlo *mc;
OdometryModel *om;
RangeModel *irm;
struct PoseState odomState;
std::vector<RangeModel::Reading> irReadings;

void odomRangeUpdate(const nav_msgs::Odometry::ConstPtr& odom_msg, const ir_sensors::RangeArray::ConstPtr& ir_msg)
{
    mtx.lock();
    current_time = odom_msg->header.stamp;
    odomState.set(0.0);
    odomState.x = odom_msg->pose.pose.position.x;
    odomState.y = odom_msg->pose.pose.position.y;
    odomState.yaw = tf::getYaw(odom_msg->pose.pose.orientation);
    
    irReadings.clear();
    struct RangeModel::Reading s;
    geometry_msgs::PoseStamped ps, pg;
    ps.header.stamp = ir_msg->header.stamp;
    ps.pose.position.y = 0;
    ps.pose.position.z = 0;
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    
    for(int i = 0; i < ir_msg->array.size(); i++){
        s.sigma = irSigma * ir_msg->array[i].max_range;
        double r = ir_msg->array[i].range;
        if(r < ir_msg->array[i].min_range || r > ir_msg->array[i].max_range){
            s.x = 0.0;
            s.y = 0.0;
            irReadings.push_back(s);
            continue;
        }
        ps.header.frame_id = ir_msg->array[i].header.frame_id;
        ps.pose.position.x = r;
        tf::StampedTransform transform;
        try{
            tf_listener->waitForTransform(baseFrame, ir_msg->array[i].header.frame_id, ir_msg->header.stamp, ros::Duration(0.5));
            tf_listener->transformPose(baseFrame, ps, pg);
        }catch(tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            mtx.unlock();
            return;
        }
        s.x = pg.pose.position.x;
        s.y = pg.pose.position.y;
        irReadings.push_back(s);
    }
    mtx.unlock();
    
}


bool updateMap(void)
{
    map_tools::GetMap srv;
    srv.request.type.data = "distance";
    if (map_client->call(srv)){
        *mapDistance = srv.response.map;
    }else{
        ROS_ERROR("Failed to call service GetMap (distance).");
        return false;
    }
    map_tools::GetMap srv2;
    srv2.request.type.data = "inflated";
    if (map_client->call(srv2)){
        *mapInflated = srv2.response.map;
    }else{
        ROS_ERROR("Failed to call service GetMap (inflated).");
        return false;
    }
    return true;
}

void initMcl(const geometry_msgs::Pose::ConstPtr& p)
{
    double csz = mapInflated->info.resolution;
    double wc = ((double)mapInflated->info.width);
    double hc = ((double)mapInflated->info.height);
    assert(csz > 0.00001);
    if(p->position.z < 0.01){
        struct PoseState pose;
        pose.set(0);
        pose.x = p->position.x;
        pose.y = p->position.y;
        pose.yaw = tf::getYaw(p->orientation);
        mc->init(pose, initConeRadius, initYawVar, wc*csz, hc*csz);
        ROS_INFO("MCL init pose.");
    }else{
        mc->init(wc*csz, hc*csz);
        ROS_INFO("MCL init unknown.");
    }
    mclEnabled = true;
}

void runMonteCarlo(void)
{
    mtx.lock();
    irm->updateMeasurements(irReadings);
    mtx.unlock();
    double csz = mapInflated->info.resolution;
    assert(csz > 0.00001);
    mc->run(odomState, ((double)mapInflated->info.width)*csz,
                        ((double)mapInflated->info.height)*csz);
    coords.x = coords.x0;
    coords.y = coords.y0;
    coords.th = 0.0;
}

int getMapValue(nav_msgs::OccupancyGrid& m, double x, double y)
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

double getDist(double x, double y)
{
    int v = getMapValue(*mapDistance, x, y);
    if(v < 0)
        return -10.0;
    return (v * mapDistance->info.resolution);
}

bool isPointFree(double x, double y)
{
    int v = getMapValue(*mapInflated, x, y);
    if(v < 0)
        return false;
    return (v < minOccupied);
}

void publishTransform(void)
{
     //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped map_trans;
    mtx.lock();
    map_trans.header.stamp = current_time;
    mtx.unlock();
    map_trans.header.frame_id = mapFrame;
    map_trans.child_frame_id = odomFrame;

    map_trans.transform.translation.x = coords.x;
    map_trans.transform.translation.y = coords.y;
    map_trans.transform.translation.z = 0.0;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(coords.th);
    map_trans.transform.rotation = quat;

    //send the transform
    tf_broadcaster->sendTransform(map_trans);
}

void particles2PoseArray(const std::vector<MonteCarlo::StateW> &particles, geometry_msgs::PoseArray &pa)
{
    geometry_msgs::Pose p;
    pa.poses.clear();
    for(int i = 0; i<particles.size(); i++){
        p.position.x = particles[i].s.x;
        p.position.y = particles[i].s.y;
        p.position.z = particles[i].s.z;
        tf::quaternionTFToMsg(
            tf::createQuaternionFromRPY(particles[i].s.roll, particles[i].s.pitch, particles[i].s.yaw),
            p.orientation);
        pa.poses.push_back(p);
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mcl_node");
	ros::NodeHandle n("/mcl_node");;
	
    n.param<std::string>("map_frame", mapFrame, "map");
    n.param<std::string>("robot_base_link", baseFrame, "base_link");
    n.param<double>("map_offset_x", coords.x0, 0.0);
    n.param<double>("map_offset_y", coords.y0, 0.0);
    n.param<int>("map_occupied_min_threshold", minOccupied, 10);
    
    double odom_a1, odom_a2, odom_a3, odom_a4;
    n.param<std::string>("odometry_frame", odomFrame, "odom");
    n.param<double>("odometry_model_a1", odom_a1, 0.05);
    n.param<double>("odometry_model_a2", odom_a2, 0.01);
    n.param<double>("odometry_model_a3", odom_a3, 0.02);
    n.param<double>("odometry_model_a4", odom_a4, 0.01);
    
    //TODO: setup these IR constants more properly.
    double irZhit, irZrm;
    n.param<double>("ir_model_sigma_hit", irSigma, 0.1);
    n.param<double>("ir_model_z_hit", irZhit, 0.95);
    n.param<double>("ir_model_z_random_over_z_max", irZrm, 0.05);
    
    int nParticles, rate;
    double minDelta, aslow, afast;
    double crashRadius, crashYaw, stdXY, stdYaw;
    n.param<int>("mcl_particles", nParticles, 200);
    n.param<int>("mcl_rate", rate, 5);
    n.param<double>("mcl_init_cone_radius", initConeRadius, 0.2);
    n.param<double>("mcl_init_yaw_variance", initYawVar, 0.3);
    n.param<double>("mcl_min_position_delta", minDelta, 0.001);
    n.param<double>("mcl_aslow", aslow, 0.01);
    n.param<double>("mcl_afast", afast, 0.2);
    n.param<double>("mcl_crash_radius", crashRadius, 0.1);
    n.param<double>("mcl_crash_yaw", crashYaw, 0.2);
    n.param<double>("mcl_good_std_xy", stdXY, 0.1);
    n.param<double>("mcl_good_std_yaw", stdYaw, 0.6);

    tf::TransformBroadcaster broadcaster_obj;
    tf_broadcaster = &broadcaster_obj;
    tf::TransformListener listener_obj;
    tf_listener = &listener_obj;
    
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "/odom", 1);
    message_filters::Subscriber<ir_sensors::RangeArray> range_sub(n,"/ir_publish/sensors", 1);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), odom_sub, range_sub);
    sync.registerCallback(boost::bind(&odomRangeUpdate, _1, _2));
    
    ros::Publisher particle_pub_obj = n.advertise<geometry_msgs::PoseArray>("/mcl/particles", 5);
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = mapFrame;
    
    //Wait 8 s for the map service.
    if(!ros::service::waitForService("/map_node/get_map", 8000)){ 
        ROS_ERROR("Map service unreachable.");
        return -1;
    }
    mapInflated = new nav_msgs::OccupancyGrid();
    mapDistance = new nav_msgs::OccupancyGrid();
    ros::ServiceClient map_client_obj = n.serviceClient<map_tools::GetMap>("/map_node/get_map");
    map_client  = &map_client_obj;
    
    if(!updateMap()){
        return -1;
    }

    struct PoseState pose, goodStd;
    goodStd.set(stdXY);
    goodStd.yaw = stdYaw;
    pose.set(0.0);
    pose.x += coords.x0;
    pose.y += coords.y0;
    om = new OdometryModel(odom_a1, odom_a2, odom_a3, odom_a4);
    
    mclEnabled = false;
    coords.x = coords.x0;
    coords.y = coords.y0;
    coords.th = 0.0;
    
    mc = new MonteCarlo(om, &isPointFree, nParticles, minDelta,
                        aslow, afast, crashRadius, crashYaw, goodStd);
    irm = new RangeModel(&getDist, irZhit, irZrm);
    mc->addSensor(irm);
    
    ros::Subscriber init_mcl_sub = n.subscribe<geometry_msgs::Pose>("/mcl/initial_pose", 2, initMcl);
    

    current_time = ros::Time::now();

    ros::Rate loop_rate(rate);
    
	while (ros::ok())
	{
        if(!updateMap()){
            return -1;
        }
        if(mclEnabled){
            runMonteCarlo();
            particles2PoseArray(mc->getParticles(), poseArray);
            particle_pub_obj.publish(poseArray);
        }
        publishTransform();
		ros::spinOnce();
		loop_rate.sleep();
        
        ros::Duration last_update = ros::Time::now() - current_time;
        if(last_update > ros::Duration(0.15))
            current_time = ros::Time::now();
	}


	return 0;
}
