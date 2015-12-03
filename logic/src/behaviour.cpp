#include "logic/FunctionBlocks.h"

int safetyTime = 20; 
const int explorationTimeout = 300/*300*/, fetchingTimeout = 180;
geometry_msgs::Pose *startPose;
FunctionBlocks *fb;
const double radiusTolerance = 0.055, yawTolerance = 2*M_PI;

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
    
    ROS_INFO("Getting out !");
    fb->go2goal(*startPose); 
    while (ros::ok()) {
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
      goal = fb->exploreNext();
      fb->go2goal(goal);
      goalSet = true;
    }
    if(fb->poseReached(goal, radiusTolerance, yawTolerance)){
      goalSet = false;
    }
    if(fb->objectDetected()){
      ROS_INFO("111111111111111111111111111111111111111111111111111111");
      fb->stopRobotAStar();
      objectArray = fb->processObject();
      ROS_INFO("222222222222222222222222222222222222222222222222222222");   
      //      fb->add2map(objectArray);
      ROS_INFO("333333333333333333333333333333333333333333333333333333");   
      fb->sendEvidence(objectArray);
      ROS_INFO("444444444444444444444444444444444444444444444444444444");   
      fb->go2goal(goal);
      ROS_INFO("555555555555555555555555555555555555555555555555555555");   
    }
    if(!fb->isLocalized()){
      fb->stopRobotAStar();
      localize();
    }
    ros::spinOnce();
    loop_rate.sleep();
  } while( (ros::ok()) && (safetyTime + fb->time2goal(*startPose) < fb->secondsLeft()) );
  getOut();
}

void exploreWall(void)
{
  const int rate = 5;
  classification::ClassifiedObjectArray objectArray;
  ros::Rate loop_rate(rate);
    
  fb->initPose(*startPose);
  fb->setWallFollower(true);
  fb->openDoor();
  fb->startTimer(explorationTimeout);
  do{
    fb->setWallFollower(true);
    if(fb->objectDetected()){
      ROS_INFO("Object detected behaviour");
      fb->setWallFollower(false);
      objectArray = fb->processObject();
      fb->add2map(objectArray);
      fb->sendEvidence(objectArray);
      fb->setWallFollower(true);
    } else {
      ROS_INFO("Nothing");
    }
    ros::spinOnce();
    loop_rate.sleep();
  } while( (ros::ok()) && (safetyTime < fb->secondsLeft()) );
  fb->setWallFollower(false);
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
      fb->stopRobotAStar();
      objectArray = fb->processObject();
      fb->add2map(objectArray);
      fetchAndReport(objectArray);
    }
    if(!fb->isLocalized()){
      fb->stopRobotAStar();
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
  startPose->position.x = 0.2;
  startPose->position.y = 0.2;
  startPose->orientation = tf::createQuaternionMsgFromYaw(0);
  
  fb = new FunctionBlocks(n);

  std::string behaviour;
  n.param<std::string>("logic_behaviour", behaviour, "explore");
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
    geometry_msgs::Pose test_pose;
    test_pose.position.x = 2.05;
    test_pose.position.y = 0.7;
    test_pose.position.z = 0;
    test_pose.orientation.x = 0;
    test_pose.orientation.y = 0;
    test_pose.orientation.z = 0;
    test_pose.orientation.w = 1;
    std::cout << "Size of the test path: " <<  fb->getPath(test_pose).poses.size() << std::endl;
     std::cout << "Distance test path " <<  fb->dist2goal(test_pose) << std::endl;
    }else if(behaviour.compare("explore_wall") == 0){
        exploreWall();
  }else{
    ROS_ERROR("Logic: Non-existent behaviour selected.");
  }
  return 0;
}
