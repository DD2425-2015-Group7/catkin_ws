#include "logic/FunctionBlocks.h"

FunctionBlocks::FunctionBlocks(ros::NodeHandle& n)
{
    this->timeout = 0;
    srand (time(NULL));
    n.param<int>("map_occupied_min_threshold", this->minOccupied, 10);
    init_mcl_pub = new ros::Publisher();
    *init_mcl_pub = n.advertise<geometry_msgs::Pose>("/mcl/initial_pose", 2, true); //use latch
    
    n.param<std::string>("map_frame", MapFrameName, "map"); 
    n.param<std::string>("robot_base_link", RobotFrameName, "base_link");

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
    
    //Wait 1 s for the add objects to map service.
    if(!ros::service::waitForService("/map_node/object_storage", 1000)){ 
        ROS_ERROR("Object storage service unreachable.");
        return;
    }

    
    //Wait 2 s for the path test service
    if(!ros::service::waitForService("/Astar/pathTest", 2000)){ 
      ROS_ERROR("GetPath service unreachable.");
      return;
    }

    //Wait 3 s for the points path execution
    if(!ros::service::waitForService("/motion_controllers/PathPointsExec", 3000)){ 
      ROS_ERROR("PathPointsExec service unreachable.");
      return;
    }
   
    objectsVision = new classification::ClassifiedObjectArray();
    objectsMap = new classification::ClassifiedObjectArray();
    objects2visit = new classification::ClassifiedObjectArray();
    justStarted = true;
    
    mapInflated = new nav_msgs::OccupancyGrid();
    map_client = new ros::ServiceClient();
    *map_client = n.serviceClient<map_tools::GetMap>("/map_node/get_map");
    add_objects_client = new ros::ServiceClient();
    *add_objects_client = n.serviceClient<map_tools::AddObjects>("/map_node/add_objects");
    object_storage_client = new ros::ServiceClient();
    *object_storage_client = n.serviceClient<map_tools::ObjectStorage>("/map_node/object_storage");
    updateMap();
    
    getPath_client = new ros::ServiceClient();
    *getPath_client = n.serviceClient<path_planning::GetPath>("/Astar/pathTest");

    getPathPoints_client = new ros::ServiceClient();
    *getPathPoints_client = n.serviceClient<motion_controllers::GetPathPoints>("/motion_controllers/PathPointsExec", false);

    mclReady = false;
    mclLocalized = false;
    
    all_markers = new visualization_msgs::MarkerArray();
    currentPath = new nav_msgs::Path();
    currentPath->header.frame_id = MapFrameName;
    
    path_pub = new ros::Publisher();
    state_marker_pub = new ros::Publisher();
    espeak_pub = new ros::Publisher();
    vision_sub = new ros::Subscriber();
    odom_sub = new ros::Subscriber();
    mcl_sub = new ros::Subscriber();
    wallfol_pub = new ros::Publisher();
    evidence_pub = new ros::Publisher();
    twist_pub = new ros::Publisher();
    goalPose_pub = new ros::Publisher();
    *path_pub =  n.advertise<nav_msgs::Path>("/logic/path", 10);
    *state_marker_pub =  n.advertise<visualization_msgs::MarkerArray>("/logic/state", 1000);
    *espeak_pub =  n.advertise<std_msgs::String>("/espeak/string", 1000);
    *vision_sub =  n.subscribe<classification::ClassifiedObjectArray>("/classifier/objects", 1000, &FunctionBlocks::visionCB, this);
    *odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, &FunctionBlocks::odomCB, this);
    *mcl_sub = n.subscribe<std_msgs::Bool>("/mcl/is_localized", 1, &FunctionBlocks::mclCB, this);
    *wallfol_pub = n.advertise<std_msgs::Bool>("/wallFollower", 1000);
    *evidence_pub = n.advertise<ras_msgs::RAS_Evidence>("/evidence", 1000);
    *twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    *goalPose_pub = n.advertise<geometry_msgs::Pose>("/goal_pose",1000);

    
    tf_listener = new tf::TransformListener();
    
    countObjDetected = 0;

    odomReady = false;

    objDetectTimeout = 25;
}
    
void FunctionBlocks::visionCB(const classification::ClassifiedObjectArray::ConstPtr& msg) {
  ROS_INFO("CALLBACK VISION");
  static int count = 0;
  // Copy the frame_id of the header
  objectsVision->header.frame_id = msg->header.frame_id; 

  if (count > 12) {
    if ( msg->objects.size() < 5 ) {
      //      objectsVision->objects.clear();
    }
    count = 0;
  }
  count++;
  
  for (int i=0; i < msg->objects.size(); i++) {
    objectsVision->objects.push_back(msg->objects[i]);
  }
}

void FunctionBlocks::mclCB(const std_msgs::Bool::ConstPtr& msg) {
  mclReady = true;
  mclLocalized = msg->data;
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
  //ROS_INFO("Turn: %f\n", yaw);
  const int rate = 10;
  ros::Rate loop_rate(rate);
  int timout = 5;
  int i = 0;
  float a_speed = 1.5;

  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.angular.z = 0;

  twist_pub->publish(twist);

  geometry_msgs::Pose initial = odomPose;
  
  double init_yaw = tf::getYaw(initial.orientation);
  if (yaw ==0) 
    return;
  int gain = (yaw > 0)? 1 : -1;

  double diff = tf::getYaw(odomPose.orientation) - init_yaw;
  //ROS_INFO("Difference turning%f\n", diff);

  do {
    //ROS_INFO("Difference turning%f\n", diff);
   
    diff = fabs(tf::getYaw(odomPose.orientation) - init_yaw);
    if (diff > M_PI) {
      diff = fmod(diff, M_PI);
    } else if (diff < -M_PI) {
      diff = -1*fmod(fabs(diff),M_PI);
    }
 
    twist.angular.z = /*fabs(diff) **/ gain * a_speed; //smoothUpdateVelocity(twist.angular.z, a_speed, 0.1);
    //ROS_INFO("Publish:%f\n", twist.angular.z);
    twist_pub->publish(twist);
    ros::spinOnce();
    loop_rate.sleep();  
    i++;
  } while ( (ros::ok()) /*&& (i < (rate*timout))*/
	    && (fabs(diff) < fabs(yaw)) );

  //ROS_INFO("End turn");
  twist.angular.z = 0;
  twist_pub->publish(twist);
}

classification::ClassifiedObjectArray FunctionBlocks::processObject(void)
{
  //TODO: process also point p2_debris for debris.
  ROS_INFO("Processing start");
  classification::ClassifiedObject lastSeen = objectsVision->objects[objectsVision->objects.size()-1];
  this->speak("Object detected"); 
  this->speak(lastSeen.name);
  std::cout << "Last object seen: coordinates: y= " << lastSeen.p.y << "x= " << lastSeen.p.x << std::endl;
  double angle = atan2(lastSeen.p.y, lastSeen.p.x);
  std::cout << "Let's turn  " << angle << std::endl;
  this->turn(angle);
  ROS_INFO("Turn OK!");

  // We reset the Array
  objectsVision->objects.clear();

  classification::ClassifiedObjectArray verifiedObjects;
  
  verifiedObjects.header.frame_id = objectsVision->header.frame_id;

  const int rate = 10;
  ros::Rate loop_rate(rate);
  int time2wait = 2; // in seconds
  int i = 0;
  
  do {
    ros::spinOnce();
    i++;
    loop_rate.sleep();
  } while ( (ros::ok()) && (i < (rate*time2wait)) ); // 2s to load the vision object 
  
  ROS_INFO("Waiting time finished");
  
  std::unordered_map<std::string,int> nbrObj;
  std::unordered_map<std::string, int>::iterator it_nbr;
  classification::ClassifiedObject current;
  std::string classID;
  std::stringstream streamx, streamy;

  //Now, let's check what the objectVion Array contains
  for (int j=0; j < objectsVision->objects.size() ; j++) {
    // if the class of the current object is not present yet
    
    current = objectsVision->objects[j];
    // Create the ID of the object
    // ClassID composed by the class of the oject, and the first 2 digits of the x and t coordinates // 1cm precision on x and y for the cells
    // 1cm precision on x and y for the cells
    streamx << std::fixed << std::setprecision(3) << current.p.x;
    streamy << std::fixed << std::setprecision(3) << current.p.y;
    classID = current.name + streamx.str() + streamy.str();
    
    it_nbr = nbrObj.find(classID);
    
    if ( it_nbr  == nbrObj.end() ) {
      // We initialize the number of object of this class
      nbrObj.insert(
		    {{
			classID,
			  1
			  }}
		    );
    } else {
      // else, we increment the counter
      nbrObj[classID] = nbrObj[classID] + 1;
    }
  }
  
  int objectsThreshold = 5;
  
  std::unordered_map<std::string, classification::ClassifiedObject> objectsTable;
  std::unordered_map<std::string, classification::ClassifiedObject>::iterator it_obj;
  
  for (int j=0; j < objectsVision->objects.size() ; j++) {
    // if the class of the current object is not present yet
    current = objectsVision->objects[j];
    // Create the ID of the object
    // ClassID composed by the class of the oject, and the first 2 digits of the x and t coordinates // 1cm precision on x and y for the cells
    // 1cm precision on x and y for the cells
    streamx << std::fixed << std::setprecision(3) << current.p.x;
    streamy << std::fixed << std::setprecision(3) << current.p.y;
    classID = current.name + streamx.str() + streamy.str();
    
    it_obj = objectsTable.find(classID);
    
    // We test if the number of object of this type present is big enough
    if ( nbrObj[current.name] > objectsThreshold) {
      if ( it_obj  == objectsTable.end() ) {
	objectsTable.insert(
			    {{
				classID, 
				  current
				  }}
			    );
      } else {
        
	/******** TEST Don't do anything ********/
	// //Average of the point
	// objectsTable[current.name].p.x = objectsTable[current.name].p.x + current.p.x;
	// objectsTable[current.name].p.y = objectsTable[current.name].p.y + current.p.y;
	// objectsTable[current.name].p.z = objectsTable[current.name].p.y + current.p.z;
	// //Average of the second debris point
	// objectsTable[current.name].p2_debris.x = objectsTable[current.name].p2_debris.x + current.p2_debris.x;
	// objectsTable[current.name].p2_debris.y = objectsTable[current.name].p2_debris.y + current.p2_debris.y;
	// objectsTable[current.name].p2_debris.z = objectsTable[current.name].p2_debris.y + current.p2_debris.z;
	// //Average of the bounding box
	// objectsTable[current.name].bb.x0 = objectsTable[current.name].bb.x0 + current.bb.x0;
	// objectsTable[current.name].bb.x1 = objectsTable[current.name].bb.x1 + current.bb.x1;
	// objectsTable[current.name].bb.y0 = objectsTable[current.name].bb.y0 + current.bb.y0;
	// objectsTable[current.name].bb.y1 = objectsTable[current.name].bb.y1 + current.bb.y1;
	// // The other attributes of the ClassifiedObject can be taken from any object of the array

      }
    }
  }

  // for (  auto it = nbrObj.begin(); it != nbrObj.end(); ++it ) {
  //   objectsTable[it->first].p.x = objectsTable[current.name].p.x / it->second;
  //   objectsTable[it->first].p.y = objectsTable[current.name].p.y / it->second;
  //   objectsTable[it->first].p.z = objectsTable[current.name].p.y / it->second;
      
  //   objectsTable[it->first].bb.x0 = objectsTable[current.name].bb.x0 / it->second; 
  //   objectsTable[it->first].bb.x1 = objectsTable[current.name].bb.x1 / it->second;
  //   objectsTable[it->first].bb.y0 = objectsTable[current.name].bb.y0 / it->second;
  //   objectsTable[it->first].bb.y1 = objectsTable[current.name].bb.y1 / it->second;
  // }
  

  // We reset the Array
  objectsVision->objects.clear();

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
  //ROS_INFO("Add2Map");
  geometry_msgs::PointStamped p0, p1, p20, p21;
  //assert(objects.header.frame_id.compare("cam_link")==0);
  p0.header.frame_id = objects.header.frame_id;
  p0.header.stamp = objects.header.stamp;
  p20.header.frame_id = objects.header.frame_id;
  p21.header.stamp = objects.header.stamp;
  
  try{
    tf_listener->waitForTransform(MapFrameName, p0.header.frame_id, p0.header.stamp, ros::Duration(1.0) );
  }catch(tf::TransformException &ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  
  objects.header.frame_id = MapFrameName;
  for(int i = 0; i < objects.objects.size(); i++){
    p0.point = objects.objects[i].p;
    p20.point = objects.objects[i].p2_debris;
    try{
      tf_listener->transformPoint(MapFrameName, p0, p1);
      tf_listener->transformPoint(MapFrameName, p20, p21);
    }catch(tf::TransformException &ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
    objects.objects[i].p = p1.point;
    objects.objects[i].p2_debris = p21.point;
    setViewPose(objects.objects[i]);
  }
    
  this->sendObjects(objects);
  this->updateMap();
}

void FunctionBlocks::testAdd2Map(void)
{
    ras_msgs::RAS_Evidence evidence;
    classification::ClassifiedObjectArray objects;
    classification::ClassifiedObject obj;
    objects.header.frame_id = "cam_link";
    objects.header.stamp = ros::Time(0);
    
    obj.name = evidence.an_object;
    obj.id = 0;
    obj.p.x = 0.8;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = evidence.red_cube;
    obj.id = 1;
    obj.p.x = 0.4;
    obj.p.y = 0.5;
    objects.objects.push_back(obj);
    
    obj.name = evidence.blue_cube;
    obj.id = 3;
    obj.p.x = 0.3;
    obj.p.y = 0.5;
    objects.objects.push_back(obj);
    
    obj.name = evidence.green_cube;
    obj.id = 4;
    obj.p.x = 0.4;
    obj.p.y = 0.7;
    objects.objects.push_back(obj);
    
    obj.name = evidence.yellow_cube;
    obj.id = 5;
    obj.p.x = 0.3;
    obj.p.y = 0.7;
    objects.objects.push_back(obj);
    
    obj.name = evidence.yellow_ball;
    obj.id = 6;
    obj.p.x = 1.0;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = evidence.red_ball;
    obj.id = 7;
    obj.p.x = 1.2;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = evidence.green_cylinder;
    obj.id = 8;
    obj.p.x = 1.4;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = evidence.blue_triangle;
    obj.id = 9;
    obj.p.x = 1.6;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = evidence.purple_cross;
    obj.id = 10;
    obj.p.x = 1.8;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = evidence.purple_star;
    obj.id = 11;
    obj.p.x = 2.0;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = evidence.patric;
    obj.id = 12;
    obj.p.x = 0.6;
    obj.p.y = 0.0;
    objects.objects.push_back(obj);
    
    obj.name = "debris";
    obj.id = 13;
    obj.p.x = 0.4;
    obj.p.y = 0.0;
    obj.p2_debris.x = 0.65;
    obj.p2_debris.y = 0.0;
    objects.objects.push_back(obj);
    
    this->add2map(objects);
}

bool FunctionBlocks::objectDetected(void)
{
  int threshold_vision = 1;

  if (objDetectTimeout > 24) {
    ROS_INFO("Detection resulat: %d\n", ((int)objectsVision->objects.size() > threshold_vision));
    return (objectsVision->objects.size() > threshold_vision);
  } else {
    objDetectTimeout++; 
    ROS_INFO("Detection TROP TOT");
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
        if(justStarted){
            *objects2visit = *objectsMap;
            justStarted = false;
        }
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

nav_msgs::Path FunctionBlocks::getPath(geometry_msgs::Pose &p){
  path_planning::GetPath srv;
  srv.request.goal = p;
  if (getPath_client->call(srv)){
    return (srv.response.path);
  } else {
    ROS_ERROR("Failed to call service GetPath");
  }
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

   nav_msgs::Path path =  getPath(p);

   for(int i = 1; (i < path.poses.size()); i++)
     {
       distance += sqrt(
			pow((path.poses[i-1].pose.position.y - path.poses[i].pose.position.y), 2) +
			pow((path.poses[i-1].pose.position.x - path.poses[i].pose.position.x), 2)
			);
     }

   return distance;
 }
    
int FunctionBlocks::time2goal(geometry_msgs::Pose &p)
{
  int time = 0;
  double distance = 0.0;
  double av_lin_vel = 0.125;
  int time2turn = 1; 

  nav_msgs::Path path =  getPath(p);
  
  for(int i = 1; i < path.poses.size(); i++)
    {
      distance += sqrt(
		       pow((path.poses[i-1].pose.position.y - path.poses[i].pose.position.y), 2) +
		       pow((path.poses[i-1].pose.position.x - path.poses[i].pose.position.x), 2)
		       );
    }

  time = (distance * av_lin_vel) + (path.poses.size() * time2turn); 

  return time;

}

void FunctionBlocks::testPathPlanning(void)
{
    geometry_msgs::Pose test_pose;
    test_pose.position.x = 2.05;
    test_pose.position.y = 0.7;
    test_pose.position.z = 0;
    test_pose.orientation.x = 0;
    test_pose.orientation.y = 0;
    test_pose.orientation.z = 0;
    test_pose.orientation.w = 1;
    nav_msgs::Path path = getPath(test_pose);
    *currentPath = path;
    std::cout << "Size of the test path: " <<  path.poses.size() << std::endl;
    std::cout << "Distance test path " <<  dist2goal(test_pose) << std::endl;
}

 bool FunctionBlocks::poseReached(geometry_msgs::Pose &pose, double radius, double yaw)
 {
   std::string TargetFrameName = "/map";
   std::string CurrentFrame = "/base_link";

   geometry_msgs::PoseStamped p, pbis;
   p.header.frame_id = CurrentFrame;
   p.header.stamp = ros::Time(0);
   p.pose.position.x = 0.0;
   p.pose.position.y = 0.0;
   p.pose.position.z = 0.0;
   p.pose.orientation.x = 0.0;
   p.pose.orientation.y = 0.0;
   p.pose.orientation.y = 0.0;
   p.pose.orientation.z = 1.0;

   try
     {
       tf_listener->waitForTransform(TargetFrameName, CurrentFrame, ros::Time(0), ros::Duration(1.0) );
       tf_listener->transformPose(TargetFrameName,p,pbis);
     }
   catch(tf::TransformException &ex)
     {
       ROS_ERROR("%s",ex.what());
     }

   double odom_yaw = tf::getYaw(pbis.pose.orientation);
   double goal_yaw = tf::getYaw(pose.orientation);

   double distance = sqrt(
			 pow(pose.position.x - pbis.pose.position.x, 2) +
			 pow(pose.position.y - pbis.pose.position.y, 2)
			 );

   //distance = dist2goal(p) if Astar faster
   double angle = fabs(goal_yaw - odom_yaw);
   return ( (distance < radius) && ( angle < yaw) );
 }

void FunctionBlocks::go2goal(geometry_msgs::Pose &p)
{

  nav_msgs::Path path =  getPath(p);
  *currentPath = path;

  motion_controllers::GetPathPoints srv;
  srv.request.path = path;
  if (!(getPathPoints_client->call(srv))) {
    ROS_ERROR("Failed to call service GetPathPoints to go to a goal");
  }
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
    for(int i = 1; i < 5; i++){
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
    p.orientation.x = p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1.0;
    if(objects2visit->objects.size() < 1)
        return p;
    p = objects2visit->objects[0].viewPose;
    double minTime = time2goal(p);
    double t;
    int chosen = 0;
    for(int i = 0; i < objects2visit->objects.size(); i++){
        t = time2goal(objects2visit->objects[i].viewPose);
        if(t < minTime){
            minTime = t;
            p = objects2visit->objects[i].viewPose;
            chosen = i;
        }
    }
    objects2visit->objects.erase(objects2visit->objects.begin()+chosen);
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

    evidence_pub->publish(evidence);
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
    if(!mclReady)
        return false;
    return mclLocalized;
}

void FunctionBlocks::objects2localize(classification::ClassifiedObjectArray &)
{
    
}

// int FunctionBlocks::objectMapped(classification::ClassifiedObject &)
// {
//     return -1;
// }

void FunctionBlocks::stopRobotAStar(void)
{
  geometry_msgs::Pose stop;
  stop.position.x = -1.0;
  stop.position.y = -1.0;
  stop.position.z = -1.0;
  stop.orientation.x = -1.0;
  stop.orientation.y = -1.0;
  stop.orientation.z = -1.0;
  stop.orientation.w = -1.0;
  
  go2goal(stop);

}

void FunctionBlocks::reportState(std::string text, int verbose)
{
    std::string head = "Behaviour: ";
    
    if(verbose>2){
        ROS_INFO("%s", (head + text).c_str());
        return;
    }
    
    all_markers->markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = MapFrameName;
    marker.header.stamp = ros::Time();
    marker.ns = "state_texts";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.pose.position.z = 0.05;
    
    marker.pose.position.x = -0.2;
    marker.pose.position.y = 1.8;
    marker.id = 1;
    marker.text = head + text;
    ROS_INFO("%s", marker.text.c_str());
    all_markers->markers.push_back(marker);
    
    marker.pose.position.x = -0.4;
    marker.pose.position.y = 1.8;
    marker.id = 2;
    marker.text = isLocalized()?"Is localized.":"Not localized.";
    ROS_INFO("%s", marker.text.c_str());
    all_markers->markers.push_back(marker);
    
    publishing();
    
}

//Periodically call this publishing function to facilitate easier debugging and visualizations.
void FunctionBlocks::publishing(void)
{
    // State texts, current path the robot is following.
    this->state_marker_pub->publish(*all_markers);
    this->path_pub->publish(*currentPath);
}

void FunctionBlocks::testReporting(void)
{
    ros::Rate loop_rate(5);
    reportState("Test behaviour.", 1);
    while(ros::ok()){
        publishing();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool FunctionBlocks::loadObjects(std::string bagFile)
{
    map_tools::ObjectStorage srv;
    srv.request.action = "load";
    srv.request.bag_file = bagFile;
    if (object_storage_client->call(srv)){
        return true;
    }else{
        ROS_ERROR("Failed to call service ObjectStorage (load).");
        return false;
    }
}

bool FunctionBlocks::saveObjects(std::string bagFile)
{
    map_tools::ObjectStorage srv;
    srv.request.action = "store";
    srv.request.bag_file = bagFile;
    if (object_storage_client->call(srv)){
        return true;
    }else{
        ROS_ERROR("Failed to call service ObjectStorage (store).");
        return false;
    }
}

