#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include "detection/BoundingBox.h"
#include "detection/BoundingBoxArray.h"
#include "classification/ClassifiedObjectArray.h"
#include "classification/ClassifiedObject.h"
#include "plane_extraction/BoundingBox_Float.h"
#include "plane_extraction/BoundingBox_FloatArray.h"
#include "sensor_msgs/Image.h"
#include <math.h>
#include <tf/transform_listener.h>

class bb_comparison{

    ros::NodeHandle n;
    tf::TransformListener listener;
    ros::Publisher bb_pub;
    ros::Publisher comp_p_pub;
    classification::ClassifiedObjectArray obj_array;
public:
    void run(){
        using namespace message_filters;
        typedef message_filters::sync_policies::ApproximateTime<plane_extraction::BoundingBox_FloatArray, geometry_msgs::PolygonStamped, sensor_msgs::Image> Policy;



        message_filters::Subscriber<plane_extraction::BoundingBox_FloatArray> bb_3D_vision(n,"/pointCloud_detector/bounding_boxes", 30);
        message_filters::Subscriber<geometry_msgs::PolygonStamped> centroid_array(n, "/object_point/uncompared",30);
        message_filters::Subscriber<sensor_msgs::Image> image(n, "/object_detection/Image",30);
        Synchronizer<Policy> sync(Policy(50),bb_3D_vision, centroid_array, image);

        bb_pub = n.advertise<detection::BoundingBoxArray> ("/compared_bounding_boxes", 30);
        comp_p_pub = n.advertise<classification::ClassifiedObjectArray>("/classifier/objects",1);
        //std::cerr << "before callback " << std::endl;
        sync.registerCallback(boost::bind(&bb_comparison::comp_callback,this, _1, _2,_3));

        ros::Rate loop_rate(5);


        obj_array.header.frame_id = "cam_link";

        ros::spin ();
        loop_rate.sleep();
    }
    void comp_callback(const  plane_extraction::BoundingBox_FloatArray::ConstPtr&  msg_b, const geometry_msgs::PolygonStamped::ConstPtr& msg_c, const sensor_msgs::Image::ConstPtr& msg_d){
        float x;
        float y;
        //std::cerr << "in the callback " << std::endl;
        //bild = msg_d; //error here

        tf::StampedTransform transform;

        try{
            //listener.lookupTransform("cam_link", "base_link",
              //                       ros::Time(0), transform);
            listener.waitForTransform("cam_link", "base_link", ros::Time::now(), ros::Duration(3.0));
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();

        }
        for(int i = 0; i < msg_b->bounding_boxes.size(); i++){
            classification::ClassifiedObject obj;
            geometry_msgs::Point pkt;
            std::string color;

            x = float(msg_c->polygon.points[i].x);
            y = float(msg_c->polygon.points[i].y);
            float x_temp = x;
            x = cos(atan2(y,x_temp));
          //  std::cerr << "x is: " << x << std::endl;
            x_temp = x;
           // std::cerr << "origin is " << transform.getOrigin().x();
            x = x_temp+transform.getOrigin().x();
            pkt.x = x;
            pkt.y = y;
            pkt.z = 0;
            obj.p = pkt;
            float size_debris_length = 0.0745;
            float size_debris_width = 0.032;
            geometry_msgs::Point p;
            geometry_msgs::Point p2;
            int32_t id;
           // std::cerr << "color: "<< msg_b->bounding_boxes[i].prob << std::endl;
            switch(int(msg_b->bounding_boxes[i].prob)){
            case 1:
                color = "Red Cube";
                id = 1;
                break;
            case 2:
                color = "Red Hollow Cube";
                id = 2;
                break;
            case 3:
                color = "Blue Cube";
                id = 3;
                break;
            case 4:
                color = "Green Cube";
                id = 4;
                break;

            case 5:
                color = "Yellow Cube";
                id = 5;
                break;
            case 6:
                color = "Yellow Ball";
                id = 6;
                break;
            case 7:
                color = "Red Ball";
                id = 7;
                break;
            case 8:
                color = "Green Cylinder";
                id = 8;
                break;
            case 9:
                color = "Blue Triangle";
                id = 9;
                break;
            case 10:
                color = "Purple Cross";
                id  = 10;
                break;
            case 11:
                color = "Purple Star";
                id = 11;
                break;
            case 12:
                color = "Patric";
                id = 12;
                break;
            case 13:
                color = "debris";//"gray";
                id = 13;
                p.x = x;
                p.y = y-size_debris_length;
                p2.x = x;
                p2.y = y+size_debris_length;
                break;
            case 14:
                color = "debris";
                id = 13;
                p.x = x+size_debris_length;
                p.y = y;
                p2.x = x-size_debris_length;
                p2.y = y;
                break;


            }
            obj.name = color;
            obj.image = *msg_d;
            obj.id = id;

            obj.p2_debris = p2;
            obj.p = p;
            obj_array.objects.push_back(obj);
        }
        if(!(obj_array.objects.size()==0)){
            comp_p_pub.publish(obj_array);
        }
    }
};



int main(int argc, char** argv){
    ros::init(argc,argv, "bb_comparison");

    bb_comparison comp;

    comp.run();



}
