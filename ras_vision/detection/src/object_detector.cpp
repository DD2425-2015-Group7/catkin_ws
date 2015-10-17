#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "detection/LinearClassifier.h"
#include "detection/MovingWindow.h"

LinearClassifier *linClass;
MovingWindow *movWin;
CImg<my_float> *img;

CImg<double> rosImg2CImg(sensor_msgs::Image rim)
{
    //This is NOT tested! The data order (x,y,c) may be different.
    unsigned char *_data = rim.data.data();
    int _width = rim.width;
    int _height = rim.height;
    int _channels = rim.step/rim.width;
    CImg<double> _img = CImg<unsigned char> (_data, _width, _height, 1, _channels);
    return _img;
}

void testLinearClassifier(void)
{
    if(linClass->testClassifier())
        ROS_INFO("Linear classifier tested OK.\n");
    else
        ROS_INFO("Linear classifier test FAILED.\n");
}

void runDetection(void)
{
    movWin->runWindow(*img, DATA_PATCHES);
    std::vector <int> pos = linClass->getPositive(movWin->getDataPatches(), 2);
    std::vector <int> bb;
    for(int i = 0; i<pos.size(); i++){
        bb = movWin->getBoundingBox(pos[i]);
        std::cout << "[" << bb[0] << " " << bb[1] << "; " << bb[2] << " " << bb[3] << "]" << std::endl;
    }
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
    
    std::string modelFile, imageFile;
    n.param<std::string>("linear_classifier_model_file", modelFile, "");
    n.param<std::string>("detector_test_image_file", imageFile, "");
	
    linClass = new LinearClassifier(modelFile.c_str());
    
    int sc[] = {320, 240, 160, 120, 80, 60};
    std::vector<int> scales(sc, sc + sizeof(sc) / sizeof(int));
    movWin = new MovingWindow(1, 3, 1, 5, scales);
    img = new CImg<my_float> (imageFile.c_str());
    //testLinearClassifier();
    runDetection();
    
    const int rate = 20;
    ros::Rate loop_rate(rate);
    
	while (ros::ok())
	{
		ros::spinOnce(); // Run the callbacks.
		loop_rate.sleep();
	}


	return 0;
}
