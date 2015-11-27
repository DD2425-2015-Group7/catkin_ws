#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point32.h>
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

	obj_array.header.frame_id = "cam_link"; // Edited by Aîmen, so check it Jesper   

        message_filters::Subscriber<plane_extraction::BoundingBox_FloatArray> bb_3D_vision(n,"/pointCloud_detector/bounding_boxes", 30);
        message_filters::Subscriber<geometry_msgs::PolygonStamped> centroid_array(n, "/object_point/uncompared",30);
        message_filters::Subscriber<sensor_msgs::Image> image(n, "/object_detection/Image",30);
        Synchronizer<Policy> sync(Policy(50),bb_3D_vision, centroid_array, image);

        bb_pub = n.advertise<detection::BoundingBoxArray> ("/compared_bounding_boxes", 30);
        comp_p_pub = n.advertise<classification::ClassifiedObjectArray>("/classifier/objects",1);
        //std::cerr << "before callback " << std::endl;
        sync.registerCallback(boost::bind(&bb_comparison::comp_callback,this, _1, _2,_3));

        ros::Rate loop_rate(5);

        ros::spin ();
        loop_rate.sleep();
    }
    void comp_callback(const  plane_extraction::BoundingBox_FloatArray::ConstPtr&  msg_b, const geometry_msgs::PolygonStamped::ConstPtr& msg_c, const sensor_msgs::Image::ConstPtr& msg_d){
        float x;
        float y;
        //std::cerr << "in the callback " << std::endl;
        //bild = msg_d; //error here

        tf::StampedTransform transform;

        /*try{
            //listener.lookupTransform("cam_link", "base_link",
              //                       ros::Time(0), transform);
            listener.waitForTransform("cam_link", "base_link", ros::Time::now(), ros::Duration(3.0));
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();

        }*/
        for(int i = 0; i < msg_b->bounding_boxes.size(); i++){
            classification::ClassifiedObject obj;
            geometry_msgs::Point pkt;
            std::string klass;

            x = float(msg_c->polygon.points[i].x);
            y = float(msg_c->polygon.points[i].y);
            //float x_temp = x;
            //x = cos(atan2(y,x_temp));
            //std::cerr << "x is: " << x << std::endl;
            //x_temp = x;
            //std::cerr << "origin is " << transform.getOrigin().x();
            //x = x_temp+transform.getOrigin().x();
            pkt.x = x;
            pkt.y = y;
            pkt.z = 0;
            obj.p = pkt;
            if(msg_b->bounding_boxes[i].prob == 1){
                klass = "Object";
            }else{
                klass = "Debris";
            }
            obj.name = klass;
            obj.image = *msg_d;
            obj_array.objects.push_back(obj);
        }
        comp_p_pub.publish(obj_array);

    }
};



int main(int argc, char** argv){
    ros::init(argc,argv, "bb_comparison");

    bb_comparison comp;

    comp.run();



}
