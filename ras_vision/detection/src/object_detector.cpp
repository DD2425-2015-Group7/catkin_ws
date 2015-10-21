#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "detection/LinearClassifier.h"
#include "detection/MovingWindow.h"
#include "detection/BoundingBox.h"
#include "detection/BoundingBoxArray.h"

int noThreads = 4;
LinearClassifier *linClass;
MovingWindow *movWin;
ros::Publisher *bb_pub;

void rosImg2CImg(const sensor_msgs::Image::ConstPtr& rim, CImg<my_float> &_img)
{
    //This is NOT tested! The data order (x,y,c) may be different.
    int _width = rim->width;
    int _height = rim->height;
    int _channels = rim->step/rim->width;

    int size = _width*_height*_channels;
    int s1 = _width*_height;
    char * dataF2 = (char*) malloc(size);
    int k = 0;
    assert(_channels==3);

    for(int i = 0; i<size; i+=_channels){
        dataF2[k+0*s1] = rim->data[i+2];
        dataF2[k+1*s1] = rim->data[i+1];
        dataF2[k+2*s1] = rim->data[i+0];
        k++;
    }

    _img = CImg<char> (dataF2, _width, _height, 1, _channels, false); //false for is_shared
    free(dataF2);
}

void testLinearClassifier(void)
{
    if(linClass->testClassifier())
        ROS_INFO("Linear classifier tested OK.\n");
    else
        ROS_INFO("Linear classifier test FAILED.\n");
}

void runDetection(CImg<my_float> *img)
{
    movWin->runWindow(*img, DATA_PATCHES);
    std::vector <int> pos = linClass->getPositive(movWin->getDataPatches(), 2);
    std::vector <int> bb;
    for(int i = 0; i<pos.size(); i++){
        bb = movWin->getBoundingBox(pos[i]);
        std::cout << "[" << bb[0] << " " << bb[1] << "; " << bb[2] << " " << bb[3] << "]" << std::endl;
    }
}


void processImage(const sensor_msgs::Image::ConstPtr& msg)
{
    CImg<my_float> img;
    rosImg2CImg(msg, img);
    img.resize(320, 240, 1, img.depth());

    movWin->runWindow(img, DATA_PATCHES);
    std::vector< std::vector<my_float> > dataPatches = movWin->getDataPatches();
    std::vector <int> pos = linClass->getPositive(dataPatches, noThreads);
    std::vector <int> bb;
    detection::BoundingBox bb_msg;
    detection::BoundingBoxArray bb_array_msg;
    bb_array_msg.header = msg->header;
    ROS_INFO("positives %lu, bb total count %lu\n", pos.size(), dataPatches.size());
    for(int i = 0; i<pos.size(); i++){
        bb = movWin->getBoundingBox(pos[i]);
        bb_msg.x0 = bb[0];
        bb_msg.y0 = bb[1];
        bb_msg.x1 = bb[2];
        bb_msg.y1 = bb[3];
        bb_array_msg.bounding_boxes.push_back(bb_msg);
    }
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
	ros::init(argc, argv, "object_detector");
	ros::NodeHandle n("/object_detector");
    
    std::string modelFile, imageTestFile, imageTopic, bbTopic;
    n.param<std::string>("linear_classifier_model_file", modelFile, "");
    n.param<int>("linear_classifier_threads", noThreads, 1);
    n.param<std::string>("detector_test_image_file", imageTestFile, "");
    n.param<std::string>("detector_image_topic", imageTopic, "/camera/rgb/image_rect_color");
    n.param<std::string>("detector_bounding_box_topic", bbTopic, "/object_detector/bounding_boxes");

    linClass = new LinearClassifier(modelFile.c_str());
    int sc[] = {320, 240, 160, 120, 80, 60};
    std::vector<int> scales(sc, sc + sizeof(sc) / sizeof(int));
    movWin = new MovingWindow(30, 30, 6, 6, scales);

    ros::Publisher bb_pub_obj = n.advertise<detection::BoundingBoxArray>(bbTopic, 50);
    bb_pub = &bb_pub_obj;
    ros::Subscriber image_sub = n.subscribe<sensor_msgs::Image>(imageTopic, 2, processImage);
	

    //CImg<my_float> img = new CImg<my_float> (imageFile.c_str());
    //testLinearClassifier()
    
    const int rate = 20;
    ros::Rate loop_rate(rate);
    
	while (ros::ok())
	{
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
