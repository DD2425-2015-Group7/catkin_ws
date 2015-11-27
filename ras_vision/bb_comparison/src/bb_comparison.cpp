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
#include "plane_extraction/BoundingBox_Float.h"
#include "plane_extraction/BoundingBox_FloatArray.h"

class bb_comparison{

    ros::NodeHandle n;

    ros::Publisher bb_pub;
    ros::Publisher comp_p_pub;

public:
    void run(){
        using namespace message_filters;
        typedef message_filters::sync_policies::ApproximateTime<detection::BoundingBoxArray,plane_extraction::BoundingBox_FloatArray, geometry_msgs::PolygonStamped> Policy;


        message_filters::Subscriber<detection::BoundingBoxArray> bb_3D_detec(n, "/object_detector3D/bounding_boxes", 30);
        message_filters::Subscriber<plane_extraction::BoundingBox_FloatArray> bb_3D_vision(n,"/pointCloud_detector/bounding_boxes", 30);
        message_filters::Subscriber<geometry_msgs::PolygonStamped> centroid_array(n, "/object_point/uncompared",30);
        Synchronizer<Policy> sync(Policy(50), bb_3D_detec, bb_3D_vision, centroid_array);

        bb_pub = n.advertise<detection::BoundingBoxArray> ("/compared_bounding_boxes", 30);
        comp_p_pub = n.advertise<geometry_msgs::Polygon>("/object_point/compared",1);

        std::cerr << "in the initialization "<< std::endl;
        sync.registerCallback(boost::bind(&bb_comparison::comp_callback,this, _1, _2, _3));

        std::cerr << "After Register" << std::endl;

        ros::Rate loop_rate(5);

        ros::spin ();
        loop_rate.sleep();
    }
    void comp_callback(const  detection::BoundingBoxArray::ConstPtr&  msg_a, const  plane_extraction::BoundingBox_FloatArray::ConstPtr&  msg_b, const geometry_msgs::PolygonStamped::ConstPtr& msg_c){


        detection::BoundingBoxArray bb_array_msg;
        std::cerr << "In the callback!" << std::endl;
        geometry_msgs::PolygonStamped comp_centroid;
        for(int i = 0; i<msg_a->bounding_boxes.size();i++){
            for(int j = 0; j<msg_b->bounding_boxes.size(); j++){
                detection::BoundingBox bb_msg;

                if(msg_b->bounding_boxes[j].x0<msg_a->bounding_boxes[i].x0<msg_b->bounding_boxes[j].x1){
                    if(msg_b->bounding_boxes[j].y0<msg_a->bounding_boxes[i].y0<msg_b->bounding_boxes[j].y1){
                        //DO STUFF, store the bb, centroid etc...
                        std::cerr << "We are in the first if" << std::endl;
                        bb_msg = msg_a->bounding_boxes[i];
                        comp_centroid.polygon.points.push_back(msg_c->polygon.points[j]);
                        bb_array_msg.bounding_boxes.push_back(bb_msg);
                    }
                }else if(msg_b->bounding_boxes[j].x0<msg_a->bounding_boxes[i].x1<msg_b->bounding_boxes[j].x1){
                    if(msg_b->bounding_boxes[j].y0<msg_a->bounding_boxes[i].y1<msg_b->bounding_boxes[j].y1){
                        //DO STUFF, store the bb, centroid etc...
                        std::cerr << "We are in the second if" << std::endl;
                        bb_msg = msg_a->bounding_boxes[i];
                        comp_centroid.polygon.points.push_back(msg_c->polygon.points[j]);
                        bb_array_msg.bounding_boxes.push_back(bb_msg);
                    }
                }else if(msg_a->bounding_boxes[j].x0<msg_b->bounding_boxes[i].x0<msg_a->bounding_boxes[j].x1){
                    if(msg_a->bounding_boxes[j].y0<msg_b->bounding_boxes[i].y0<msg_a->bounding_boxes[j].y1){
                        //DO STUFF, store the bb, centroid etc...
                        std::cerr << "We are in the third if" << std::endl;
                        bb_msg = msg_a->bounding_boxes[i];
                        comp_centroid.polygon.points.push_back(msg_c->polygon.points[j]);
                        bb_array_msg.bounding_boxes.push_back(bb_msg);
                    }
                }else if(msg_a->bounding_boxes[j].x0<msg_b->bounding_boxes[i].x1<msg_a->bounding_boxes[j].x1){
                    if(msg_a->bounding_boxes[j].y0<msg_b->bounding_boxes[i].y1<msg_a->bounding_boxes[j].y1){
                        //DO STUFF, store the bb, centroid etc...
                        std::cerr << "We are in the fourth if" << std::endl;
                        bb_msg = msg_a->bounding_boxes[i];
                        comp_centroid.polygon.points.push_back(msg_c->polygon.points[j]);
                        bb_array_msg.bounding_boxes.push_back(bb_msg);
                    }
                }
            }
        }
        bb_pub.publish(bb_array_msg);
    }
};



int main(int argc, char** argv){
    ros::init(argc,argv, "bb_comparison");

    bb_comparison comp;

    comp.run();



}
