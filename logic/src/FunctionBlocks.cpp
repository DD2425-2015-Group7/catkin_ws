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
    
    //Wait 2 s for the add objects to map service.
    if(!ros::service::waitForService("/map_node/add_objects", 2000)){ 
        ROS_ERROR("Add objects to map service unreachable.");
        return;
    }
    
    //Wait 2 s for the add objects to map service.
    if(!ros::service::waitForService("/Astart/pathTest", 2000)){ 
      ROS_ERROR("GetPath service unreachable.");
      return;
    }
    
    objectsVision = new classification::ClassifiedObjectArray();
    objectsMap = new classification::ClassifiedObjectArray();
    
    mapInflated = new nav_msgs::OccupancyGrid();
    map_client = new ros::ServiceClient();
    *map_client = n.serviceClient<map_tools::GetMap>("/map_node/get_map");
    add_objects_client = new ros::ServiceClient();
    *add_objects_client = n.serviceClient<map_tools::AddObjects>("/map_node/add_objects");
    updateMap();
    
    getPath_client = new ros::ServiceClient();
    *getPath_client = n.serviceClient<path_planning::GetPath>("/Astar/pathTest");


    espeak_pub = new ros::Publisher();;
    vision_sub = new ros::Subscriber();
    odom_sub = new ros::Subscriber();
    wallfol_pub = new ros::Publisher();
    evidence_pub = new ros::Publisher();
    twist_pub = new ros::Publisher();
    goalPose_pub = new ros::Publisher();
    *espeak_pub =  n.advertise<std_msgs::String>("/espeak/string", 1000);
    *vision_sub =  n.subscribe<classification::ClassifiedObjectArray>("/classifier/objects", 1000, &FunctionBlocks::visionCB, this);
    *odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &FunctionBlocks::odomCB, this);
    *wallfol_pub = n.advertise<std_msgs::Bool>("/wallFollower", 1000);
    *evidence_pub = n.advertise<ras_msgs::RAS_Evidence>("/evidence", 1000);
    *twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    *goalPose_pub = n.advertise<geometry_msgs::Pose>("/goal_pose",1000);

    n.param<std::string>("map_frame", MapFrameName, "map"); 
    n.param<std::string>("robot_base_link", RobotFrameName, "base_link");
    
    tf_listener = new tf::TransformListener();
    
    countObjDetected = 0;

    odomReady = false;

    objDetectTimeout = 25;
}
    
void FunctionBlocks::visionCB(const classification::ClassifiedObjectArray::ConstPtr& msg) {
  static int count = 0;
  
  if (count > 12) {
    if ( msg->objects.size() < 5 ) {
      objectsVision->objects.clear();
    }
    count = 0;
  }
  count++;

   for (int i=0; i < msg->objects.size(); i++) {
      objectsVision->objects.push_back(msg->objects[i]);
    }
}

void FunctionBlocks::odomCB(const nav_msgs::Odometry::ConstPtr& msg) {
  odomReady = true;
  odomPose = msg->pose.pose;
}

double FunctionBlocks::smoothUpdateVelocity(double current, double required, double step)
{
    double diff = current - required;
    if(fabs(diff) > 1.5*step)
    {
       current -= (diff > 0) ? step : -step;
    }
    else
    {
        current = required;
    }
    return current;
}

void FunctionBlocks::turn(double yaw) 
{
  const int rate = 10;
  ros::Rate loop_rate(rate);
  int timout = 2;
  int i = 0;
  float a_speed = 1.5;

  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.angular.z = 0;

  twist_pub->publish(twist);

  geometry_msgs::Pose initial = odomPose;
  
  double init_yaw = tf::getYaw(initial.orientation);
  

  while(tf::getYaw(odomPose.orientation) - init_yaw < yaw) {
    twist.angular.z = smoothUpdateVelocity(twist.angular.z, a_speed, 0.1);
    twist_pub->publish(twist);
    ros::spinOnce();
    i++;
    loop_rate.sleep();
  } while ( (ros::ok()) && (i < (rate*timout)) );

  twist.angular.z = 0;
  twist_pub->publish(twist);
}

classification::ClassifiedObjectArray FunctionBlocks::processObject(void)
{
   this->speak("Object detected");
  classification::ClassifiedObject lastSeen = objectsVision->objects[objectsVision->objects.size()-1];
  double angle = atan2(lastSeen.p.y, lastSeen.p.x);
  this->turn(angle);

   const int rate = 10;
   ros::Rate loop_rate(rate);
   int time2wait = 2; // in seconds
   int i = 0;

   do {
     ros::spinOnce();
     i++;
     loop_rate.sleep();
   } while ( (ros::ok()) && (i < (rate*time2wait)) ); // 2s to load the vision object 
  
 
   std::unordered_map<std::string,int> nbrObj;
   std::unordered_map<std::string, int>::iterator it_nbr;
   classification::ClassifiedObject current;
   
   //Now, let's check what the objectVion Array contains
   for (int j=0; j < objectsVision->objects.size() ; j++) {
     // if the class of the current object is not present yet
     current = objectsVision->objects[j];
     it_nbr = nbrObj.find(current.name);
     
     if ( it_nbr  == nbrObj.end() ) {
       // We initialize the number of object of this class
       nbrObj.insert(
		     {{
			 current.name, 
			   1
			   }}
		     );
     } else {
       // else, we increment the counter
       nbrObj[current.name] = nbrObj[current.name] + 1;
     }
   }
   
   int objectsThreshold = 1000;
   
   std::unordered_map<std::string, classification::ClassifiedObject> objectsTable;
   std::unordered_map<std::string, classification::ClassifiedObject>::iterator it_obj;

   for (int j=0; j < objectsVision->objects.size() ; j++) {
     // if the class of the current object is not present yet
     current = objectsVision->objects[j];
     it_obj = objectsTable.find(current.name);
     
     // We test if the number of object of this type present is big enough
     if ( nbrObj[current.name] > objectsThreshold) {
       if ( it_obj  == objectsTable.end() ) {
   	 objectsTable.insert(
   			     {{
   				 current.name, 
   				   current
   				   }}
   			     );
       } else {
   	 //Average of the point
   	 objectsTable[current.name].p.x = objectsTable[current.name].p.x + current.p.x;
   	 objectsTable[current.name].p.y = objectsTable[current.name].p.y + current.p.y;
   	 objectsTable[current.name].p.z = objectsTable[current.name].p.y + current.p.z;
   	 //Average of the bounding box
   	 objectsTable[current.name].bb.x0 = (1/2) * (objectsTable[current.name].bb.x0 + current.bb.x0);
   	 objectsTable[current.name].bb.x1 = (1/2) * ( objectsTable[current.name].bb.x1 + current.bb.x1);
   	 objectsTable[current.name].bb.y0 = (1/2) * (objectsTable[current.name].bb.y0 + current.bb.y0);
   	 objectsTable[current.name].bb.y1 = (1/2) * (objectsTable[current.name].bb.y1 + current.bb.y1);
	 // The other attributes of tege ClassifiedObject can be taken from any object of the array
       }
     }
   }
   
   // We reset the Array
   objectsVision->objects.clear();
     
   classification::ClassifiedObjectArray verifiedObjects;
   
   for (it_obj = objectsTable.begin(); it_obj != objectsTable.end(); ++it_obj ) {
     verifiedObjects.objects.push_back(it_obj->second);
   }
   
   objDetectTimeout = 0;

   return verifiedObjects;
}

void FunctionBlocks::setViewPose(classification::ClassifiedObject& obj)
{
    
}

void FunctionBlocks::add2map(classification::ClassifiedObjectArray& objects)
{
    geometry_msgs::PointStamped p0, p1;
    p0.header.frame_id = objects.header.frame_id;
    p0.header.stamp = objects.header.stamp;
    
    try{
      tf_listener->waitForTransform(MapFrameName, p0.header.frame_id, p0.header.stamp, ros::Duration(1.0) );
    }catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
    
    objects.header.frame_id = MapFrameName;
    for(int i = 0; i < objects.objects.size(); i++){
        p0.point = objects.objects[i].p;
        try{
          tf_listener->transformPoint(MapFrameName, p0, p1);
        }catch(tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            return;
        }
        objects.objects[i].p = p1.point;
        setViewPose(objects.objects[i]);
    }
    
    this->sendObjects(objects);
    this->updateMap();
}

void FunctionBlocks::testAdd2Map(void)
{
    classification::ClassifiedObjectArray objects;
    classification::ClassifiedObject obj;
    objects.header.frame_id = "cam_link";
    objects.header.stamp = ros::Time::now();
    obj.name = "red_cube";
    obj.id = 1;
    obj.p.x = 0.4;
    obj.p.y = 0.5;
    objects.objects.push_back(obj);
    obj.name = "debris";
    obj.id = 13;
    obj.p.x = 0.4;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    this->add2map(objects);
}

bool FunctionBlocks::objectDetected(void)
{
  int threshold_vision = 25;

  if (objDetectTimeout > 24) {
    return (objectsVision->objects.size() > threshold_vision);
  } else {
    objDetectTimeout++; 
    return false;
  }

}

bool FunctionBlocks::sendObjects(classification::ClassifiedObjectArray& objects)
{
    map_tools::AddObjects srv;
    srv.request.array = objects;
    if (!add_objects_client->call(srv)){
        ROS_ERROR("Failed to call service AddObjects.");
        return false;
    }
    return true;
}

bool FunctionBlocks::updateMap(void)
{
    map_tools::GetMap srv2;
    srv2.request.type.data = "inflated_obj";
    if (map_client->call(srv2)){
        *mapInflated = srv2.response.map;
        *objectsMap = srv2.response.mappedObjects;
        mapXsz = mapInflated->info.width * mapInflated->info.resolution;
        mapYsz = mapInflated->info.height * mapInflated->info.resolution;
    }else{
        ROS_ERROR("Failed to call service GetMap (inflated_obj).");
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

 double FunctionBlocks::dist2goal(geometry_msgs::Pose &p)
 {
   double distance = 0.0;

   path_planning::GetPath srv;
   srv.request.goal = p;
   if (getPath_client->call(srv)){
       for(int i = 1; (i < srv.response.path.poses.size()); i++)
	 {
	   distance += sqrt(
			    pow((srv.response.path.poses[i-1].pose.position.y - srv.response.path.poses[i].pose.position.y), 2) +
			    pow((srv.response.path.poses[i-1].pose.position.x - srv.response.path.poses[i].pose.position.x), 2)
			    );
	 }
     return distance;
   } else {
     ROS_ERROR("Failed to call service GetPath for the distance");
   }
 }
    
int FunctionBlocks::time2goal(geometry_msgs::Pose &p)
{
  int time = 0;
  double distance = 0.0;
  double av_lin_vel = 0.125;
  int time2turn = 1; 

  path_planning::GetPath srv;
  srv.request.goal = p;
  if (getPath_client->call(srv)){
      for(int i = 1; i < srv.response.path.poses.size(); i++)
	{
	  distance += sqrt(
			   pow((srv.response.path.poses[i-1].pose.position.y - srv.response.path.poses[i].pose.position.y), 2) +
			   pow((srv.response.path.poses[i-1].pose.position.x - srv.response.path.poses[i].pose.position.x), 2)
			   );
	}

    time = (distance * av_lin_vel) + (srv.response.path.poses.size() * time2turn); 

    return time;
  } else {
    ROS_ERROR("Failed to call service GetPath for the time");
  }
}

 bool FunctionBlocks::poseReached(geometry_msgs::Pose &p, double radius, double yaw)
 {
   double odom_yaw = tf::getYaw(odomPose.orientation);
   double goal_yaw = tf::getYaw(p.orientation);

   double distance = sqrt(
			 pow(p.position.x - odomPose.position.x, 2) +
			 pow(p.position.y - odomPose.position.y, 2)
			 );

   //distance = dist2goal(p) if Astar faster
   double angle = fabs(goal_yaw - odom_yaw);
   return ( (distance < radius) && ( angle < yaw) );
 }

void FunctionBlocks::go2goal(geometry_msgs::Pose &p)
{
  goalPose_pub->publish(p);
}

void FunctionBlocks::setWallFollower(bool on)
{
  std_msgs::Bool wf;
  wf.data = on;
  ros::Rate loop_rate(5);
  loop_rate.sleep();
  wallfol_pub->publish(wf);
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
  std_msgs::String msg; 
  msg.data = text; 
  this->espeak_pub->publish(msg);
}

void FunctionBlocks::sendEvidence(classification::ClassifiedObjectArray& objArray)
{
  ras_msgs::RAS_Evidence evidence;
  for (int i = 0; i < objArray.objects.size(); i++) {
    evidence.stamp = ros::Time::now();
    evidence.group_number = 7;
    evidence.image_evidence = objArray.objects[i].image;
    evidence.object_id = objArray.objects[i].name;
    evidence.object_location.transform.translation.x = objArray.objects[i].p.x;
    evidence.object_location.transform.translation.y = objArray.objects[i].p.y;
  }
}

void FunctionBlocks::openDoor(void)
{
    this->speak("Starting");
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
    /*
    sleep(2);
    initUnknown();
    sleep(2);
    initPose(pose);
    sleep(2);
    initPose(pose);
    */
}

bool FunctionBlocks::isLocalized(void)
{
    return true;
}

void FunctionBlocks::objects2localize(classification::ClassifiedObjectArray &)
{
    
}

// int FunctionBlocks::objectMapped(classification::ClassifiedObject &)
// {
//     return -1;
// }
