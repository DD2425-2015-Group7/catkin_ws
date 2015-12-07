#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include "detection/BoundingBox.h"
#include "detection/BoundingBoxArray.h"
#include <pcl/common/centroid.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PolygonStamped.h>
#include "plane_extraction/BoundingBox_Float.h"
#include "plane_extraction/BoundingBox_FloatArray.h"
#include <math.h>
#include <vector>

class Plane_Extraction{

    ros::NodeHandle n;

    ros::Publisher c_pub;
    ros::Publisher obj_loc_pub;
    ros::Publisher bb_publish;
    ros::Publisher i_pub;
    std::string color_path;
    std::vector<std::vector<int> > class_color_avg;
    int nmr_classes;
    std::vector<std::string> color_v;
public:

    void run(){
        using namespace message_filters;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image> Policy;
        //n.param<std::string>("~average_file", color_path, "");
        ros::param::get("~average_file",color_path);
        c_pub = n.advertise<sensor_msgs::PointCloud2> ("/clustered_object_cloud", 1);
        obj_loc_pub = n.advertise<geometry_msgs::PolygonStamped>("/object_point/uncompared",20);
        //bb_publish =  n.advertise<plane_extraction::BoundingBox_FloatArray>("/pointCloud_detector/bounding_boxes", 50); //detection::BoundingBoxArray>
        i_pub = n.advertise<sensor_msgs::Image> ("/object_detection/Image",20);

        message_filters::Subscriber<sensor_msgs::PointCloud2> Cloud(n, "/camera/depth_registered/points", 30);
        message_filters::Subscriber<sensor_msgs::Image> image(n, "/camera/rgb/image_rect_color",30);
        Synchronizer<Policy> sync(Policy(50),Cloud, image);

        sync.registerCallback(boost::bind(&Plane_Extraction::cloud_callback,this, _1, _2));
        ros::Rate loop_rate(15);

        ros::spin ();
        loop_rate.sleep();
    }
    float e_dist(std::vector<int> c_ref, std::vector<int> c_local)
    {
        float Sum;
        float distance;
        //std::cerr << "The reference's size: " << c_ref.size() << std::endl;
        for(int i=0;i<c_ref.size();i++)
        {
            Sum = Sum + (c_ref[i]-c_local[i])*(c_ref[i]-c_local[i]);
            distance = sqrt(Sum);
        }

        return distance;
    }
    void reader(const char avgFile[]){
        //initialize avg, weights, bias,
        class_color_avg.clear();
        nmr_classes = 16;
        std::ifstream ifs(avgFile);
        assert(ifs.is_open());
        int ff;
        int dim = 3;
        try {

            for(int i = 0; i<nmr_classes; i++){
                std::vector<int> temp;
                for(int j = 0; j<dim; j++){
                    ifs>>ff;
                    temp.push_back(ff);

                }
                class_color_avg.push_back(temp);

            }
        } catch (...){

            std::cerr << "error in load" << std::endl;

        }

    }

    void PassFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,int xmin,int xmax,int ymin,int ymax){

        pcl::PassThrough<pcl::PointXYZRGB> filtery;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr smaller_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud = *input;

        float ymin1 = -0.1;
        float ymax1 = 0.06; //0.12
        float xmin1 = -0.16;
        float xmax1 = 0.16;

        filtery.setFilterFieldName("y");
        filtery.setFilterLimitsNegative (false);
        filtery.setInputCloud(input);

        filtery.setFilterLimits(ymin1,ymax1);

        filtery.filter(*smaller_cloud);

        filtery.setFilterFieldName("x");
        filtery.setFilterLimitsNegative (false);
        filtery.setInputCloud(smaller_cloud);

        filtery.setFilterLimits(xmin1,xmax1);

        filtery.filter(*output);


    }
    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg, const sensor_msgs::Image::ConstPtr& image_msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered_blob(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB> cloud_blob_test;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr init_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(*msg,*init_cloud);
        pcl::fromROSMsg(*msg,cloud_blob_test);

        sensor_msgs::Image i_to_pub; //publish the image...


        reader(color_path.c_str());

        // std::cerr << "msg_b->bounding_boxes.size() "<< msg_b->bounding_boxes.size() << std::endl;

        double x_width;
        double y_width;
        float x_0, y_0;
        float x_1, y_1;
        int isObject;

        PassFilter(init_cloud, cloud_blob, 0, 192,60, 580);
        // build the condition
        //sensor_msgs::PointCloud2 out;

        //pcl::toROSMsg(*cloud_blob, out);
        //c_pub.publish(out);

        // cloud_blob = init_cloud;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
        // Create the filtering object: downsample the dataset using a leaf size of 1cm when 0.01f on all. When using it for our objects 1cm is too big and 1 mm is too small for the amount of particles that we have.
        //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_blob);
        sor.setLeafSize(0.006f, 0.006f, 0.006f);
        sor.filter(*cloud_filtered_blob);

        cloud_filtered = cloud_filtered_blob;

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // Create the segmentation object

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional

        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.01);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        int nr_points = (int)cloud_filtered->points.size();
        // While 10% of the original cloud is still there
        while (cloud_filtered->points.size() > 0.2 * nr_points)
        {
           // std::cerr << "Entered Plane While "<< std::endl;

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                std::cerr << "Could not estimate a planar mode   l for the given dataset." << std::endl;
                break;
            }

            // Extract the inliers
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_p);


            // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

            // Create the filtering object
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
        }



        if(cloud_filtered->width*cloud_filtered->height != 0 ){

            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance (0.008); // 1cm
            ec.setMinClusterSize (100);
            ec.setMaxClusterSize (400);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            //find objects and put them in a new separate point cloud. only takes the objects that consist of 900-3000 particles
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            if(cluster_indices.begin() == cluster_indices.end()){
               // std::cerr << "samma" << std::endl;
            }else{
                int j = 0;
                plane_extraction::BoundingBox_FloatArray bbox_array_msg;
                geometry_msgs::PolygonStamped point_array;
                 //std::cerr << "==========================New Set====================================" << std::endl;
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                {
                    std::cerr << "==========================New Cluster====================================" << std::endl;
                    //uncomment this and comment the previous declaration if you only want the "newest" object to be found
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

                    int r = 0;
                    int g = 0;
                    int b = 0;

                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //
                        cloud->points.push_back(cloud_filtered->points[*pit]);
                        r = r+cloud_filtered->points[*pit].r;
                        g = g+cloud_filtered->points[*pit].g;
                        b = b+cloud_filtered->points[*pit].b;

                    }
                    r = r/cloud_cluster->points.size();
                    g = g/cloud_cluster->points.size();
                    b = b/cloud_cluster->points.size();

                    std::string color;

                   // int index = 0;
                    int R = int(r);
                    //int RG = int((r + g)/2);
                    int G = int(g);
                   // int GB = int((g+b)/2);
                    int B = int(b);
                    //int BR = int((b+r)/2);
                    //int RGB = int((r+g+b)/3);

                    std::vector<int> cluster_color;

                    cluster_color.push_back(R);

                    cluster_color.push_back(G);

                    cluster_color.push_back(B);


                    float current_dist;
                    float dist = 85;
                    float current_max = 30;
                    int color_index = 12;
                    float temp_to_print = 255;
                   for(int o = 0; o<class_color_avg.size();o++){
                       current_dist = e_dist(class_color_avg[o], cluster_color);
                       if(temp_to_print>current_dist){
                           temp_to_print = current_dist;
                       }
                       if(current_dist < dist ){
                       //    if(current_dist<current_max){
                               current_max = current_dist;
                               color_index = o;



                         //  }else{
                           //    color_index = 13;

                          // }

                       }
                   }

                   switch(color_index){
                   case 0:
                       color = "Red Cube";
                       isObject = 1;
                       break;
                   case 1:
                       color = "Red Hollow Cube";
                       isObject = 2;
                       break;
                   case 2:
                       color = "Blue Cube";
                       isObject = 3;
                       break;
                   case 3:
                       color = "Green Cube";
                       isObject = 4;
                       break;
                   case 4:
                       color = "Yellow Cube";
                       isObject = 5;
                       break;
                   case 5:
                       color = "Yellow Ball";
                       isObject =6;
                       break;
                   case 6:
                       color = "Red Ball";
                       isObject = 7;
                       break;
                   case 7:
                       color = "Green Cylinder";
                       isObject = 8;
                       break;
                   case 8:
                       color = "Blue Triangle";
                       isObject = 9;
                       break;
                   case 9:
                       color = "Purple Cross";
                       isObject = 10;
                       break;
                   case 10:
//                     color = "Purple Star";
                       color = "garbage";
                       isObject = 11;
                       break;
                   case 11:
                       color = "Patric";
                       isObject = 12;
                       break;
                   case 12:
                       color = "garbage";
                       if(cloud_cluster->points.size()<350){
                           isObject = 14;
                       }else if(cloud_cluster->points.size() > 350){
                           isObject = 13;
                       } else{
                           isObject = 14;
                       }
                       break;
                   case 13:
                       color = "garbage";
                       isObject = 17;
                       break;
                   case 14:
                       color = "garbage";
                       isObject = 18;
                       break;
                   case 15:
                       color = "garbage";
                       isObject = 19;
                       break;
                   }
                    std::cerr << "clasS: " << isObject << std::endl;

                    if(color != "garbage"){
                        sensor_msgs::PointCloud2 out;

                       // if(color != "debris"){
                            std::cerr << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

                            std::cerr << "the Error is: " << temp_to_print << std::endl;

                            std::cerr<<"color: "<< color<<std::endl;
                            cloud_cluster->header = cloud_filtered->header;
                            pcl::toROSMsg(*cloud_cluster, out);
                            c_pub.publish(out);
                        //}
                        Eigen::Vector4f centroid;
                        geometry_msgs::Point p;
                        pcl::compute3DCentroid(*cloud_cluster, centroid);
                        p.x = (double) centroid[0];
                        p.y = (double) centroid[1];
                        p.z = (double) centroid[2];

                        if(centroid[2] < 1 ){
                            // build the condition



                            x_width = cloud_cluster->width + 20;
                            y_width = cloud_cluster->height + 20;

                            //Determine the dimension of the bounding box ===========================

                            if(cloud_cluster->points.size() > 100){

                               /* Eigen::Vector4f min_pt, max_pt;
                                pcl::getMinMax3D (*cloud_filtered,it->indices, min_pt, max_pt);
                                double x_0_3D = min_pt[0];
                                double y_0_3D = min_pt[1];
                                double x_1_3D = max_pt[0];
                                double y_1_3D = max_pt[1];
                                 */

                                //std::cerr << "the points bb: "<< "("<<x_0_3D<< ", " << y_0_3D <<", " << x_1_3D << ", " << y_1_3D << ")" << std::endl;
                                plane_extraction::BoundingBox_Float bbox_msg;
                                // cloud = *cloud_cluster;
                                //std::cerr << "the points bb: "<< "("<<x_0_3D<< ", " << y_0_3D <<", " << x_1_3D << ", " << y_1_3D << ")" << std::endl;

                                // =====================================================================
                                std::vector<int> temp0;
                                std::vector<int> temp1;

                               /* x_0 = float(x_0_3D);
                                x_1 = float(x_1_3D);
                                y_0 = float(y_0_3D);
                                y_1 = float(y_1_3D);
                               // std::cerr << "after if"<< std::endl;
                                bbox_msg.x0 = x_0;
                                bbox_msg.y0 = y_0;
                                bbox_msg.x1 = x_1;
                                bbox_msg.y1 = y_1;*/
                                //std::cerr << "the points bb: "<< "("<<x_0<< ", " << y_0<<", " << x_1 << ", " << y_1 << ")" << std::endl;



                               /* if(cloud_cluster->points.size() > 5000){ //TBD
                                    isObject = 0;
                                    // std::cerr << "skräp vid: "<< "(" << centroid[0] << ", " << centroid[1] <<", " << centroid[2] << ")."<< std::endl;
                                }else{
                                    isObject = 1;
                                }*/
                                bbox_msg.prob = float(isObject);

                                bbox_array_msg.bounding_boxes.push_back(bbox_msg);

                                geometry_msgs::Point32 pkt;

                                pkt.x = centroid[2]; //in the robot coordinate, needs to be done in TF, maybe array to fit with bounding box??
                                pkt.y = centroid[0];
                                pkt.z = centroid[1];

                                pkt.x = pkt.x*cos(atan2(pkt.z,pkt.x));
                             //  std::cerr << "coordinate of cluster: (" << pkt.x << ", " << pkt.y << ", " << pkt.z << ")"<<std::endl;

                                //obj_loc_pub.publish(pkt);
                                point_array.polygon.points.push_back(pkt);
                            }
                        }
                    }
                    j++;
                }
                sensor_msgs::PointCloud2 out;


                cloud->header = cloud_filtered->header;
                //obj_array.header.frame_id = "cam_link";

                pcl::toROSMsg(*cloud, out);
                //c_pub.publish(out);

                obj_loc_pub.publish(point_array);

                i_pub.publish(i_to_pub);
                //bb_publish.publish(bbox_array_msg);
                //std::cout <<"number of clusters " << j << std::endl;
            }
        }

        // }


    }
private:
    std::vector<detection::BoundingBox> BB_Array;
    std::vector<int32_t> x0;
    std::vector<int32_t> x1;
    std::vector<int32_t> y0;
    std::vector<int32_t> y1;
};

int main(int argc, char** argv){
    ros::init(argc,argv, "Plane_Extraction");
    Plane_Extraction extracter;

    extracter.run();
    // Spin
    //vros::spin ();

}
