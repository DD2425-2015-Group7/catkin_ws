#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "detection/MovingWindow.h"
#include "detection/BoundingBox.h"
#include "detection/BoundingBoxArray.h"
#include "detection/ImageConverter.h"
#include "detection/MultiClassClassifier.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "plane_extraction/BoundingBox_Float.h"
#include "plane_extraction/BoundingBox_FloatArray.h"


int noThreads = 4, imageSaveID = 0, nmbrClass = 12;
std::string savedImagesPath;
LinearClassifier *linClass;
MultiClassifier *multiClass;
MovingWindow *movWin;
ros::Publisher *bb_pub;
ros::Publisher *bb_pub3d;
detection::BoundingBoxArray toPixel;
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
    std::vector <struct Classification> pos = linClass->getPositive(movWin->getDataPatches(), 2);
    std::vector <int> bb;
    for(int i = 0; i<pos.size(); i++){
        bb = movWin->getBoundingBox(pos[i].index);
        std::cout << "[" << bb[0] << " " << bb[1] << "; " << bb[2] << " " << bb[3] << "]" << std::endl;
    }
}


void processImage(const sensor_msgs::Image::ConstPtr& msg)
{
    CImg<my_float> img;
    rosImg2CImg(msg, img);
    img.resize(320, 240, img.depth(), img.spectrum());

    movWin->runWindow(img, DATA_PATCHES);
    std::vector< std::vector<my_float> > dataPatches = movWin->getDataPatches();
    std::vector <struct Classification> pos = linClass->getPositive(dataPatches, noThreads);
    //use index pos
    //use data patch index
    //std::vector<std::vector<my_float>> posDataPatches;
    /*for(int i  = 0; i<pos.size(); i++){
        posDataPatches.push_back(dataPatches[pos[i].index]);


    }
    //std::vector<struct Classification> pos_classify = multiClass->MVote(posDataPatches,noThreads,12);
    for(int i=0; i<pos_classify.size(); i++){
        std::cerr << "type: "<< pos_classify[i].obj_type<< std::endl;
    }*/
    std::vector <int> bb;
    detection::BoundingBox bb_msg;
    detection::BoundingBoxArray bb_array_msg;

    bb_array_msg.header = msg->header;
    //ROS_DEBUG("positives %lu, bb total count %lu\n", pos.size(), dataPatches.size());

    for(int i = 0; i<pos.size(); i++){
        bb = movWin->getBoundingBox(pos[i].index);
        bb_msg.x0 = bb[0];
        bb_msg.y0 = bb[1];
        bb_msg.x1 = bb[2];
        bb_msg.y1 = bb[3];
        bb_msg.prob = pos[i].prob;
        bb_array_msg.bounding_boxes.push_back(bb_msg);
    }
    toPixel = bb_array_msg;
    bb_pub->publish(bb_array_msg);
}

void saveImage(const sensor_msgs::Image::ConstPtr& msg)
{
    static int counter = 1;
    CImg<my_float> img;
    rosImg2CImg(msg, img);
    img.resize(320, 240, img.depth(), img.spectrum());

    std::ostringstream stringStream;
    stringStream << savedImagesPath << "/neg" << imageSaveID << "-" << counter << ".png";
    std::string cos = stringStream.str();
    std::cout << cos << std::endl;

    img.save(cos.c_str());
    counter++;
}

void bb2pixel(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    plane_extraction::BoundingBox_FloatArray bbox_msg;

    pcl::fromROSMsg(*msg,*cloud_blob);
    cloud = *cloud_blob;
     std::cerr << "size: "<<toPixel.bounding_boxes.size() << std::endl;
    for(int i=0; i< toPixel.bounding_boxes.size();i++){
        std::cerr << "size: "<<toPixel.bounding_boxes.size() << std::endl;
        std::cerr <<"index: "<< i << std::endl;

        plane_extraction::BoundingBox_Float bbox;
        //bbox.x0 = float(cloud(toPixel.bounding_boxes[i].x0,toPixel.bounding_boxes[i].y0).x);
       //bbox.y0 = float(cloud(toPixel.bounding_boxes[i].x0,toPixel.bounding_boxes[i].y0).y);
       // bbox.x1 = float(cloud(toPixel.bounding_boxes[i].x1,toPixel.bounding_boxes[i].y1).x);
     //   bbox.y1 = float(cloud(toPixel.bounding_boxes[i].x1,toPixel.bounding_boxes[i].y1).y);
        //std::cerr << "(x0,y0,x1,y1)" << bbox.x0 << ", " << bbox.y0 << ", " << bbox.x1 << ", " << bbox.y1 << ")" << std::endl;
        bbox_msg.bounding_boxes.push_back(bbox);
    }
   // bb_pub3d->publish(bbox_msg);
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
    
    std::string modelFile, imageTestFile, imageTopic, imageSaveTopic, bbTopic, multimodelFile, bbTopic3D;
    n.param<std::string>("linear_classifier_model_file", modelFile, "");
    n.param<std::string>("linear_multi_model_file", multimodelFile, "");
    n.param<int>("linear_classifier_threads", noThreads, 1);
    n.param<std::string>("detector_test_image_file", imageTestFile, "");
    n.param<std::string>("detector_image_topic", imageTopic, "/camera/rgb/image_rect_color");
    n.param<std::string>("detector_bounding_box_topic", bbTopic, "/object_detector/bounding_boxes");
    n.param<std::string>("detector_bounding_box_topic3D", bbTopic3D, "/object_detector3D/bounding_boxes");
    n.param<std::string>("save_image_topic", imageSaveTopic, "/object_detector/save_image");
    n.param<std::string>("save_image_path", savedImagesPath, "data/negative_boost");
    n.param<int>("save_image_id", imageSaveID, 0);
    if(multimodelFile == ""){
        multimodelFile = "/home/ras27/catkin_ws/src/ras_vision/detection/models/lin_class_multi_model.csv";
    }
    if(modelFile == ""){
        modelFile = "/home/ras27/catkin_ws/src/ras_vision/detection/models/lin_class_cube_model.csv";
    }
    try {
      linClass = new LinearClassifier(modelFile.c_str());
    }
    catch (const std::bad_alloc&) {
      std::cerr<< "bad_alloc";
    }

    int sc[] = {320, 240, 160, 120, 80, 60};
    std::vector<int> scales(sc, sc + sizeof(sc) / sizeof(int));
    movWin = new MovingWindow(30, 30, 6, 6, scales);

    ros::Publisher bb_pub_obj = n.advertise<detection::BoundingBoxArray>(bbTopic, 50);
    ros::Publisher bb_pub_obj3D = n.advertise<plane_extraction::BoundingBox_FloatArray>(bbTopic3D, 50);
    bb_pub = &bb_pub_obj;
    bb_pub3d = &bb_pub_obj3D;
    ros::Subscriber image_sub = n.subscribe<sensor_msgs::Image>(imageTopic, 2, processImage);
    ros::Subscriber point_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points",30, bb2pixel);

    ros::Subscriber image_save_sub = n.subscribe<sensor_msgs::Image>(imageSaveTopic, 2, saveImage);
	

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
