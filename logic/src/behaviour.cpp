#include "logic/FunctionBlocks.h"

int safetyTime = 20; 
const int explorationTimeout = 300, fetchingTimeout = 180;
geometry_msgs::Pose *startPose;
FunctionBlocks *fb;
const double radiusTolerance = 0.04, yawTolerance = 0.3;

void localize(void)
{
  const int rate = 5;
  classification::ClassifiedObjectArray objectArray;
  ros::Rate loop_rate(rate);
    
  fb->initUnknown();
  fb->setWallFollower(true);

  while (ros::ok()){
    if(fb->objectDetected()){
      objectArray = fb->processObject();
      fb->objects2localize(objectArray);
    }
    if(fb->isLocalized()){
      fb->setWallFollower(false);
      return;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  fb->setWallFollower(false);
}

void getOut(void)
{
    const int rate = 5;
    ros::Rate loop_rate(rate);
    
    while (ros::ok())
	{
        fb->go2goal(*startPose);
        if(fb->poseReached(*startPose, radiusTolerance, yawTolerance)){
            fb->speak("Hooray! I have done it! Applause, please!");
            return;
        }
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
    
  fb->initPose(*startPose);
  fb->setWallFollower(false);
  fb->openDoor();
  fb->startTimer(explorationTimeout);
  do{
    if(!goalSet){
      goal = fb->exploreNext();
      goalSet = true;
    }
    fb->go2goal(goal);
    if(fb->poseReached(goal, radiusTolerance, yawTolerance)){
      goalSet = false;
    }
    if(fb->objectDetected()){
      objectArray = fb->processObject();
      fb->add2map(objectArray);
      fb->sendEvidence(objectArray);
    }
    if(!fb->isLocalized()){
      localize();
    }
    ros::spinOnce();
    loop_rate.sleep();
  } while( (ros::ok()) && (safetyTime + fb->time2goal(*startPose) < fb->secondsLeft()) );
  getOut();
}

void fetchAndReport(classification::ClassifiedObjectArray& objectArray)
{
    const int rate = 5;
    ros::Rate loop_rate(rate);
    geometry_msgs::Pose goal = fb->fetchNext();
    //TODO: remove visited objects from the right stack and in the right function!
    //      compare fb->fetchNext() and fb->sendEvidence(objectArray)
    do{
        fb->go2goal(goal);
        if(fb->poseReached(goal, radiusTolerance, yawTolerance)){
            fb->sendEvidence(objectArray);
            return;
        }
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
    
  fb->initUnknown();
  fb->openDoor();
  fb->startTimer(fetchingTimeout);
  localize();
    
  do{
    if(!goalSet){
      goal = fb->fetchNext();
      goalSet = true;
    }
    fb->go2goal(goal);
    if(fb->poseReached(goal, radiusTolerance, yawTolerance)){
      goalSet = false;
    }
    if(fb->objectDetected()){
      objectArray = fb->processObject();
      fb->add2map(objectArray);
      fetchAndReport(objectArray);
    }
    if(!fb->isLocalized()){
      localize();
    }
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
  fb = new FunctionBlocks(n);

  std::string behaviour;
  n.param<std::string>("logic_behaviour", behaviour, "test");
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
    // fb->testTimer();
    ROS_INFO("It works!");
  }else{
    ROS_ERROR("Logic: Non-existent behaviour selected.");
  }
  return 0;
}
