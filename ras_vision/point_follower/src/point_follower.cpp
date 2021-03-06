#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

static const std::string OPENCV_WINDOW = "Image window";
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

double max = 0.0, min=0.0, min_prev = 0.0;
int count =0;
cv::Point min_loc, max_loc, min_loc_prev, min_loc_prev2, min_loc_prev3;

//this class gathers the depth data and puts it in a matrix, also plots it in a figure. TODO: find the biggest element in matrix and also location of it
class Point_Follower{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_subscriber;
    ros::Subscriber PCLsub;
    ros::Publisher PosePub;

public:
    Point_Follower()
        :it_(n)
    {
        depth_subscriber = it_.subscribe("/camera/depth_registered/image_raw", 1, &Point_Follower::callback_follow_Node,this);
        PCLsub = n.subscribe("/camera/depth/points", 1,  &Point_Follower::pub_node,this);
        PosePub = n.advertise<geometry_msgs::Point>("/next_point",1);
        //cv::namedWindow(OPENCV_WINDOW);
    }
    ~Point_Follower(){
        cv::destroyWindow(OPENCV_WINDOW);
    }


    void callback_follow_Node(const sensor_msgs::Image::ConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImagePtr cv_fptr;
        int KERNEL_LENGTH = 31;


        try {
            cv_ptr = cv_bridge::toCvCopy(msg); //copy msg to OpenCV image format...
            cv_fptr = cv_bridge::toCvCopy(msg);
            cv::Mat arr(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F); //make matrix from image, convert to different format...CV_8UC1)
            cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(7,7), 0,0);
            cv::minMaxLoc(cv_ptr->image, &min, &max, &min_loc, &max_loc); //find max and min value..
            if(!(min > 1)){
                flag = false;
                if(test == false){
                    min_loc_prev = min_loc;
                    min_prev = min;
                    test = true;

                }else{

                    if((min_loc_prev.x > 1.35*min_loc.x)||(min_loc_prev.x<0.65*min_loc.x)||(min_loc_prev.y > 1.35*min_loc.y)||(min_loc_prev.y<0.65*min_loc.y)){

                        min = min_prev;
                        min_loc_prev2 = min_loc;
                        min_loc = min_loc_prev;
                        min_loc_prev = min_loc;
                        count +=1;

                    }else
                    {
                        min_prev = min;
                        min_loc_prev2 = min_loc;
                        min_loc_prev = min_loc;

                    }

                }
                if(count > 7){

                    cv::minMaxLoc(cv_ptr->image, &min, &max, &min_loc, &max_loc);
                    min_loc_prev = min_loc;
                    count = 0;

                }
 		//cv::circle(cv_ptr->image, min_loc, 50, CV_RGB(255,0,0), 9);
                x = min_loc.x;
                y = min_loc.y;
		//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
                cv::waitKey(1);
            }else{
                flag = true;
            }
        }catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    void pub_node(const sensor_msgs::PointCloud2ConstPtr& msg){
        //Calculates x,y coordinates given depth and pixel values..
        if(flag==false){
            PointCloud cloud;
            pcl::fromROSMsg(*msg,cloud);
            geometry_msgs::Point pub_msg;
            float cloud_x = min;
            float cloud_y = cloud(x,y).x;
            float cloud_z = cloud(x,y).x;
            double cloud_x_pub;
            double cloud_y_pub;
            if(isnan(cloud_x)||isnan(cloud_y)){

                cloud_y = 0.0;
            }
            cloud_x_pub = (double) cloud_x;
            cloud_y_pub = (double) cloud_y;
            pub_msg.x = cloud_x_pub;
            pub_msg.y = cloud_y_pub;

            PosePub.publish(pub_msg);
            // std::cout << "x,y is: [" << cloud_x<<","<<cloud_z<<"]"<<std::endl;
        }
    }

    void run(){
        ros::Rate loop_rate(10);
        std::cout<<"Got data!"<<std::endl;
        while(ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
        }

    }

private:
    std::vector<uint8_t> depth_matrix;
    //uint8_t arr[640][480];
    int x = 640/2;
    int y = 480/2;
    uint32_t row;
    uint32_t column;
    float biggest=0;
    bool test = false;
    bool flag = false;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "point_follower");

    Point_Follower follow;

    follow.run();
    ros::spin();


}
