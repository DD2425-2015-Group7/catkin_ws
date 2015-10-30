#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "classification/ClassifiedObject.h"
#include "classification/ClassifiedObjectArray.h"

ros::Publisher *object_pub;


int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "object_classifier");
	ros::NodeHandle n("/object_classifier");
    
    std::string objTopic;
    n.param<std::string>("classified_object_topic", objTopic, "/classifier/objects");

    ros::Publisher obj_pub_obj = n.advertise<classification::ClassifiedObjectArray>(objTopic, 50);
    object_pub = &obj_pub_obj;
    
    const int rate = 10;
    ros::Rate loop_rate(rate);
    
	while (ros::ok())
	{
        classification::ClassifiedObjectArray coa;
        classification::ClassifiedObject obj;
        obj.id = 0;
        obj.name = "To be done.";
        obj.p.x = 0.5;
        obj.p.y = 0.2;
        obj.p.z = 0.1;
        coa.objects.push_back(obj);
        object_pub->publish(coa);
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
