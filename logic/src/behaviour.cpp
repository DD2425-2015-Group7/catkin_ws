#include "logic/FunctionBlocks.h"

int safetyTime = 20; 
const int explorationTimeout = 180/*300*/, fetchingTimeout = 180;
geometry_msgs::Pose *startPose;
FunctionBlocks *fb;
const double radiusTolerance = 0.055, yawTolerance = 2*M_PI;
std::string objects_bag;

void localize(void)
{
  const int rate = 5;
  classification::ClassifiedObjectArray objectArray;
  ros::Rate loop_rate(rate);
  
  fb->reportState("Localize.", 1);
    
  fb->initUnknown();
  fb->setWallFollower(true);

  while (ros::ok()){
    if(fb->objectDetected()){
      fb->reportState("Localize and object detected.", 2);  
      objectArray = fb->processObject();
      fb->objects2localize(objectArray);
    }
    if(fb->isLocalized()){
      fb->setWallFollower(false);
      return;
    }
    fb->publishing();
    ros::spinOnce();
    loop_rate.sleep();
  }
  fb->setWallFollower(false);
}

void getOut(void)
{
    const int rate = 5;
    ros::Rate loop_rate(rate);
    
    fb->reportState("Getting out!", 1);
    fb->go2goal(*startPose); 
    while (ros::ok()) {
      if(fb->poseReached(*startPose, radiusTolerance, yawTolerance)){
	fb->speak("Hooray! I have done it! Applause, please!");
	return;
      }
      fb->publishing();
      ros::spinOnce();
      loop_rate.sleep();
    }
}

void explore(void)
{
  const int rate = 5;
  geometry_msgs::Pose goal;
  classification::ClassifiedObjectArray objectArray;
  bool goalSet = false;
  ros::Rate loop_rate(rate);
  fb->reportState("Explore.", 1);  
  fb->initPose(*startPose);
  while (ros::ok() && (!fb->isLocalized())) {
    fb->stopRobotAStar();
    ros::spinOnce();
    loop_rate.sleep();
  }
  fb->setWallFollower(false);
  fb->openDoor();
  fb->startTimer(explorationTimeout);

  do{
    if(!goalSet){
      fb->reportState("New exploration goal.", 2);  
      goal = fb->exploreNext();
      fb->go2goal(goal);
      goalSet = true;
    }
    if(fb->poseReached(goal, radiusTolerance, yawTolerance)){
        fb->reportState("Exploration goal reached.", 2);  
      goalSet = false;
    }
    
    if(fb->objectDetected()){
      fb->reportState("Exploring and object detected.", 2);  
      fb->stopRobotAStar();
      objectArray = fb->processObject();
      fb->reportState("Exploring and object processed.", 3);     
      fb->add2map(objectArray);  
      fb->sendEvidence(objectArray);
      fb->go2goal(goal);
      fb->reportState("Exploration continues.", 2);  
    }
    if(!fb->isLocalized()){
      fb->stopRobotAStar();
      localize();
    }
    fb->publishing();
    ros::spinOnce();
    loop_rate.sleep();
  } while( (ros::ok()) && (safetyTime + fb->time2goal(*startPose) < fb->secondsLeft()) );
  fb->saveObjects(objects_bag);
  getOut();
}

void exploreWall(void)
{
  const int rate = 5;
  classification::ClassifiedObjectArray objectArray;
  ros::Rate loop_rate(rate);
    
    fb->reportState("Exploring wall.", 1);
  fb->initPose(*startPose);
  fb->setWallFollower(true);
  fb->openDoor();
  fb->startTimer(explorationTimeout);
  do{
    fb->setWallFollower(true);
    if(fb->objectDetected()){
        fb->reportState("Exploring wall and object detected.", 2);  
      fb->setWallFollower(false);
      objectArray = fb->processObject();
      fb->add2map(objectArray);
      fb->sendEvidence(objectArray);
      fb->setWallFollower(true);
    } else {
      fb->reportState("Exploring wall and nothing detected.", 2);
    }
    fb->publishing();
    ros::spinOnce();
    loop_rate.sleep();
  } while( (ros::ok()) && (safetyTime < fb->secondsLeft()) );
  fb->setWallFollower(false);
}

void fetchAndReport(classification::ClassifiedObjectArray& objectArray)
{
    const int rate = 5;
    ros::Rate loop_rate(rate);
    fb->reportState("Fetch and report an object.", 1);
    geometry_msgs::Pose goal = fb->fetchNext();
    //TODO: remove visited objects from the right stack and in the right function!
    //      compare fb->fetchNext() and fb->sendEvidence(objectArray)
    fb->go2goal(goal);
    do{
        if(fb->poseReached(goal, radiusTolerance, yawTolerance)){
            fb->reportState("Object fetched.", 1);
            fb->sendEvidence(objectArray);
            return;
        }
        fb->publishing();
        ros::spinOnce();
        loop_rate.sleep();
    }while( (ros::ok()) && (safetyTime + fb->time2goal(*startPose) < fb->secondsLeft()) );
}

void fetch(void)
{
  const int rate = 5;
  geometry_msgs::Pose goal;
  classification::ClassifiedObjectArray objectArray;
  bool goalSet = false;
  ros::Rate loop_rate(rate);
    
  fb->reportState("Fetch.", 1);
  fb->initUnknown();
  fb->loadObjects(objects_bag);
  fb->openDoor();
  fb->startTimer(fetchingTimeout);
  localize();
    
  do{
    if(!goalSet){
        fb->reportState("New fetching goal.", 2);
      goal = fb->fetchNext();
      // If there is no object left in the fetching stack, get out.
      if(goal.position.z > 1.0){
          break;
      }
      fb->go2goal(goal);
      goalSet = true;
    }
    
    if(fb->poseReached(goal, radiusTolerance, yawTolerance)){
        fb->reportState("Fetching goal reached.", 2);
      goalSet = false;
    }
    if(fb->objectDetected()){
        fb->reportState("Fetching and object detected.", 2);
      fb->stopRobotAStar();
      objectArray = fb->processObject();
      fb->add2map(objectArray);
      fetchAndReport(objectArray);
    }
    if(!fb->isLocalized()){
      fb->stopRobotAStar();
      localize();
    }
    fb->publishing();
    ros::spinOnce();
    loop_rate.sleep();
  }while( (ros::ok()) && (safetyTime + fb->time2goal(*startPose) < fb->secondsLeft()) );
  getOut();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behaviour");
  ros::NodeHandle n("/behaviour");
  
  startPose = new geometry_msgs::Pose();
  startPose->position.x = 0.2;//0.22;//0.22; //0.2;                                
  startPose->position.y = 0.2;//0.66;//2.25; //0.2; 
  startPose->orientation = tf::createQuaternionMsgFromYaw(0);
  
  fb = new FunctionBlocks(n);

  std::string behaviour;
  n.param<std::string>("logic_behaviour", behaviour, "explore");
  n.param<std::string>("objects_bag_file", objects_bag, "");
  
  if(behaviour.compare("explore") == 0){
    explore();
  }else if(behaviour.compare("fetch") == 0){
    fetch();
  }else if(behaviour.compare("test") == 0){
    // ROS_INFO("Testing...");
    // fb->testMclInit();
    // fb->testExploration();
    // fb->fetchNext();
    // fb->testAdd2Map();
    // fb->saveObjects(objects_bag);
    // fb->loadObjects(objects_bag);
    // fb->testTimer();
    // fb->testPathPlanning();
    // fb->testReporting();
    }else if(behaviour.compare("explore_wall") == 0){
        exploreWall();
  }else{
    ROS_ERROR("Logic: Non-existent behaviour selected.");
  }
  return 0;
}
