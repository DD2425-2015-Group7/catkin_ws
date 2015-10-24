#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "sensor_msgs/Image.h"
#include "detection/BoundingBox.h"
#include "detection/BoundingBoxArray.h"
#include "detection/ImageConverter.h"


ros::Publisher *view_pub;


void callback(const sensor_msgs::Image::ConstPtr& msg_img, const detection::BoundingBoxArray::ConstPtr& msg_bb)
{
    static int pattern = 1;
    assert(msg_bb->header.stamp.nsec == msg_img->header.stamp.nsec);
    assert(msg_bb->header.stamp.sec == msg_img->header.stamp.sec);
    const unsigned char colour[] = { 0,255,0 };
    CImg<my_float> img;
    rosImg2CImg(msg_img, img);
    img.resize(320, 240, img.depth(), img.spectrum());
    int x0, y0, x1, y1;

    for(int i = 0; i<msg_bb->bounding_boxes.size(); i++){
        x0 = msg_bb->bounding_boxes[i].x0;
        y0 = msg_bb->bounding_boxes[i].y0;
        x1 = msg_bb->bounding_boxes[i].x1;
        y1 = msg_bb->bounding_boxes[i].y1;
        img.draw_line(x0, y0, x0, y1, colour);
        img.draw_line(x0, y0, x1, y0, colour);
        img.draw_line(x1, y1, x0, y1, colour);
        img.draw_line(x1, y1, x1, y0, colour);
    }
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    ptr->header = msg_img->header;
    ptr->encoding = msg_img->encoding;
    ptr->is_bigendian = msg_img->is_bigendian;

    CImg2rosImg(img, *ptr);
    view_pub->publish(*ptr);
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
    ros::init(argc, argv, "show_detection");
    ros::NodeHandle n("/show_detection");
    
    std::string imageTopic, viewTopic, bbTopic;
    n.param<std::string>("detector_image_topic", imageTopic, "/camera/rgb/image_rect_color");
    n.param<std::string>("detector_bounding_box_topic", bbTopic, "/object_detector/bounding_boxes");
    n.param<std::string>("view_topic", viewTopic, "/object_detector/view");

    ros::Publisher view_pub_obj = n.advertise<sensor_msgs::Image>(viewTopic, 30);
    view_pub = &view_pub_obj;
    
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, imageTopic, 30);
    message_filters::Subscriber<detection::BoundingBoxArray> bb_sub(n, bbTopic, 30);
    message_filters::TimeSynchronizer<sensor_msgs::Image, detection::BoundingBoxArray> sync(image_sub, bb_sub, 30);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();


	return 0;
}
