#include "ros/ros.h"
#include "detection/BoundingBox.h"
#include "detection/BoundingBoxArray.h"

ros::Publisher *bb_pub;
int sequenceLength = 1;


void filterBB(const detection::BoundingBoxArray::ConstPtr& msg)
{
    detection::BoundingBoxArray bb_array_msg;
    assert(sequenceLength > 0);
    /* TODO:
     * Queue bounding boxes from multiple images.
     * Cluster the bounding boxes by location.
     * Remove singletons.
     * Return moving average/median. */
    bb_array_msg.header = msg->header;
    bb_array_msg.bounding_boxes = msg->bounding_boxes;
    bb_pub->publish(bb_array_msg);
}


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
    ros::init(argc, argv, "sequence_filter");
    ros::NodeHandle n("/sequence_filter");
    
    std::string bbDetTopic, bbFilteredTopic;

    n.param<std::string>("detector_bounding_box_topic", bbDetTopic, "/object_detector/bounding_boxes");
    n.param<std::string>("filtered_bounding_box_topic", bbFilteredTopic, "/object_detector/sequence_filtered");
    n.param<int>("sequence_filter_length", sequenceLength, 3);

    ros::Publisher bb_pub_obj = n.advertise<detection::BoundingBoxArray>(bbFilteredTopic, 50);
    bb_pub = &bb_pub_obj;
    ros::Subscriber bb_det_sub = n.subscribe<detection::BoundingBoxArray>(bbDetTopic, 50, filterBB);
    
    ros::spin();
	return 0;
}
