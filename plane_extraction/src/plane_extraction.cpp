
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
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
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include "detection/BoundingBox.h"
#include "detection/BoundingBoxArray.h"
#include <pcl/common/centroid.h>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



class Plane_Extraction{

    ros::NodeHandle n;

    ros::Publisher c_pub;
    ros::Publisher oloc_pub;
    //ros::Subscriber c_sub;
    //ros::Subscriber b_box;

public:

    void run(){
        using namespace message_filters;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,detection::BoundingBoxArray> Policy;


        message_filters::Subscriber<detection::BoundingBoxArray> b_box(n, "/object_detector/bounding_boxes", 30);
        //message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera/rgb/image_rect_color", 30);
        message_filters::Subscriber<sensor_msgs::PointCloud2> c_sub(n,"/camera/depth_registered/points", 30);

        //message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, detection::BoundingBoxArray> sync(c_sub, b_box, 30);
        Synchronizer<Policy> sync(Policy(50), c_sub, b_box);

        c_pub = n.advertise<sensor_msgs::PointCloud2> ("/clustered_object_cloud", 1);
        oloc_pub = n.advertise<geometry_msgs::Point>("/object_point",1);
        std::cerr << "in the initialization "<< std::endl;
        sync.registerCallback(boost::bind(&Plane_Extraction::cloud_callback,this, _1, _2));
       // std::cerr << sync.checkInterMessageBound() << std::endl;
        std::cerr << "After Register" << std::endl;
        /*b_box = n.subscribe("/BoundingBoxArray",1, &Plane_Extraction::Bounding_callback,this);
        c_sub = n.subscribe ("/camera/depth_registered/points", 1, &Plane_Extraction::cloud_callback, this);*/
        ros::Rate loop_rate(5);

        ros::spin ();
        loop_rate.sleep();
    }
    /*void Bounding_callback(const detection::BoundingBoxArray::ConstPtr& msg){
        msg->bounding_boxes[i].x;
        BB_Array= msg->bounding_boxes;
        for(int i=0;i<msg->bounding_boxes.size(); ++i){
            BB_Array.push
            x0.push_back(msg->bounding_boxes[i].x0);
            x1.push_back(msg->bounding_boxes[i].x1);
            y0.push_back(msg->bounding_boxes[i].y0);
            y1.push_back(msg->bounding_boxes[i].y1);
        }

    }*/
    void PassFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,int xmin,int xmax,int ymin,int ymax){
        pcl::PassThrough<pcl::PointXYZRGB> filterx;
        pcl::PassThrough<pcl::PointXYZRGB> filtery;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr smaller_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        filterx.setFilterFieldName("x");
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud = *input;
        std::cerr << "min X: " << xmin << ", max X: " << xmax << std::endl;
        std::cerr << "min y: " <<ymin<< ", max y: " << ymax<< std::endl;
        float xmin1 = cloud(xmin, ymin).x; // (270, 190) for centered bounding box...
        float xmax1 = cloud(xmax,ymax).x;  // (370,290)
        float ymin1 = cloud(xmin, ymin).y; // (270, 190)
        float ymax1 = cloud(xmax, ymax).y; // (370, 290)
        while(isnan(xmin1)||isnan(xmax1)||isnan(ymin1)||isnan(ymax1)){
            if(isnan(xmin1)){
                xmin -=1;

                xmin1 = cloud(xmin, ymin).x;
                ymin1 = cloud(xmin,ymin).y;
            }
            if(isnan(xmax1)){
                xmax+=1;
                xmax1 = cloud(xmax,ymin).x;
                ymax1 = cloud(xmax,ymin).y;
            }
            if(isnan(ymin1)){
                ymin -= 1;
                xmin1 = cloud(xmin, ymin).x;
                ymin1 = cloud(xmin,ymin).y;
            }
            if(isnan(ymax1)){
                ymax+=1;
                xmax1 = cloud(xmax,ymin).x;
                ymax1 = cloud(xmax,ymin).y;
            }
        }
        filterx.setInputCloud(input);

        std::cerr << "min Xe: " << xmin1 << ", max Xe: " << xmax1 << std::endl;
        std::cerr << "min ye: " <<ymin1<< ", max ye: " << ymax1<< std::endl;

        filterx.setFilterLimits(xmin1, xmax1);
        filterx.setFilterLimitsNegative (false);
        filterx.filter(*smaller_cloud);

        filtery.setFilterFieldName("y");
        filtery.setFilterLimitsNegative (false);
        filtery.setInputCloud(smaller_cloud);

        filtery.setFilterLimits(ymin1,ymax1);

        filtery.filter(*output);


    }

    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg, const  detection::BoundingBoxArray::ConstPtr&  msg_b)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered_blob(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB> cloud_blob_test;
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr init_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(*msg,*init_cloud);
        pcl::fromROSMsg(*msg,cloud_blob_test);

        std::cerr << "msg_b->bounding_boxes.size() "<< msg_b->bounding_boxes.size() << std::endl;

        for(int k=0; k<msg_b->bounding_boxes.size();++k){
            if(init_cloud->width*init_cloud->height == 0){
                break;
            }
            //--------------------------------------------------REMOVE ALL BUT BOUNDING BOX ------------------------------------------//
            // -1*(2*msg_b->bounding_boxes[k].x1-320), -1*(2*msg_b->bounding_boxes[k].x0-320), -1*(2*msg_b->bounding_boxes[k].y1-240),-1*(2*msg_b->bounding_boxes[k].y0-240)
            PassFilter(init_cloud, cloud_blob, 2*msg_b->bounding_boxes[k].x0, 2*msg_b->bounding_boxes[k].x1,2*msg_b->bounding_boxes[k].y0, 2*msg_b->bounding_boxes[k].y1);
            std::cerr << "PointCloud before filtering: " << init_cloud->width * init_cloud->height << " data points." << std::endl;
            std::cerr << "PointCloud after cropping: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
            //sensor_msgs::PointCloud2 out;



            /*pcl::toROSMsg(*cloud_blob, out);


            c_pub.publish(out);*/
            if(cloud_blob->width*cloud_blob->height == 0){
                break;
            }
            //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
            // Create the filtering object: downsample the dataset using a leaf size of 1cm when 0.01f on all. When using it for our objects 1cm is too big and 1 mm is too small for the amount of particles that we have.
            //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(cloud_blob);
            sor.setLeafSize(0.001f, 0.001f, 0.001f);
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

            int i = 0, nr_points = (int)cloud_filtered->points.size();
            // While 30% of the original cloud is still there
           while (cloud_filtered->points.size() > 0.3 * nr_points)
            {
                std::cerr << "Entered Plane While "<< std::endl;

                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_filtered);
                seg.segment(*inliers, *coefficients);
                if (inliers->indices.size() == 0)
                {
                    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
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
            //std::cerr << "Between IFS" << std::endl;
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance (0.05); // 10cm
            ec.setMinClusterSize (100);
            ec.setMaxClusterSize (2000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            //find objects and put them in a new separate point cloud. only takes the objects that consist of 900-3000 particles

            if(cluster_indices.begin() == cluster_indices.end()){
                std::cerr << "samma" << std::endl;
            }else{
                int j = 0;
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                {
                    //std::cerr << "IN object Clustering" << std::endl;
                    //uncomment this and comment the previous declaration if you only want the "newest" object to be found
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //

                    }
                    Eigen::Vector4f centroid;
                    geometry_msgs::Point p;
                    pcl::compute3DCentroid(*cloud_cluster, centroid);
                    p.x = (double) centroid[0];
                    p.y = (double) centroid[0];
                    p.z = (double) centroid[0];

                    if(centroid[2] < 1){

                        std::cout << "The XYZ coordinates of the centroid are: ("
                                  << centroid[0] << ", "
                                  << centroid[1] << ", "
                                  << centroid[2] << ")." << std::endl;
                        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

                    }
                    cloud_cluster->width = cloud_cluster->points.size ();
                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;


                    sensor_msgs::PointCloud2 out;


                    cloud_cluster->header = cloud_filtered->header;
                    pcl::toROSMsg(*cloud_cluster, out);


                    c_pub.publish(out);
                    j++;
                }
                std::cout <<"number of clusters " << j << std::endl;
            }
        }


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
