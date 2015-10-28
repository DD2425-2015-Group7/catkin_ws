
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

using namespace message_filters;

class Plane_Extraction{

    ros::NodeHandle n;

    ros::Publisher c_pub;
    ros::Publisher oloc_pub;
    //ros::Subscriber c_sub;
    //ros::Subscriber b_box;

public:

    void run(){

        message_filters::Subscriber<detection::BoundingBoxArray> b_box(n, "/BoundingBoxArray", 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> c_sub(n,"/camera/depth_registered/points", 1);

        message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, detection::BoundingBoxArray> sync(c_sub, b_box, 30);

        c_pub = n.advertise<sensor_msgs::PointCloud2> ("/clustered_object_cloud", 1);
        oloc_pub = n.advertise<geometry_msgs::Point>("/object_point",1);

        sync.registerCallback(boost::bind(&Plane_Extraction::cloud_callback,this, _1, _2));
        /*b_box = n.subscribe("/BoundingBoxArray",1, &Plane_Extraction::Bounding_callback,this);
        c_sub = n.subscribe ("/camera/depth_registered/points", 1, &Plane_Extraction::cloud_callback, this);*/

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

    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg, const detection::BoundingBoxArray::ConstPtr& msg_b)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered_blob(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr smaller_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB> cloud_blob_test;
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr init_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(*msg,*init_cloud);
        pcl::fromROSMsg(*msg,cloud_blob_test);

        for(int k=0; k<msg_b->bounding_boxes.size();++k){


            pcl::PassThrough<pcl::PointXYZRGB> filterx;
            pcl::PassThrough<pcl::PointXYZRGB> filtery;

            filterx.setInputCloud(init_cloud);

            filterx.setFilterFieldName("x");
            filterx.setFilterLimits(msg_b->bounding_boxes[k].x0, msg_b->bounding_boxes[k].x1);
            filterx.filter(*smaller_cloud);


            filtery.setInputCloud(smaller_cloud);
            filtery.setFilterFieldName("y");
            filtery.setFilterLimits(msg_b->bounding_boxes[k]. y0,msg_b->bounding_boxes[k].y1);
            filtery.filter(*cloud_blob);

            std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

            // Create the filtering object: downsample the dataset using a leaf size of 1cm when 0.01f on all. When using it for our objects 1cm is too big and 1 mm is too small for the amount of particles that we have.
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(cloud_blob);
            sor.setLeafSize(0.001f, 0.001f, 0.1f);
            sor.filter(*cloud_filtered_blob);
            //pcl::PCDWriter writer;
            cloud_filtered = cloud_filtered_blob;
            std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

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

                std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

                // Create the filtering object
                extract.setNegative(true);
                extract.filter(*cloud_f);
                *cloud_filtered = *cloud_f;
            }

            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance (0.05); // 5cm
            ec.setMinClusterSize (900);
            ec.setMaxClusterSize (3000);
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
                    //uncomment this and comment the previous declaration if you only want the "newest" object to be found
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
                        Eigen::Vector4f centroid;
                        geometry_msgs::Point p;
                        pcl::compute3DCentroid(*cloud_cluster, centroid);
                        p.x = (double) centroid[0];
                        p.y = (double) centroid[0];
                        p.z = (double) centroid[0];
                        std::cout << "The XYZ coordinates of the centroid are: ("
                                  << centroid[0] << ", "
                                  << centroid[1] << ", "
                                  << centroid[2] << ")." << std::endl;

                    }
                    cloud_cluster->width = cloud_cluster->points.size ();
                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;

                    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

                    sensor_msgs::PointCloud2 out;


                    cloud_cluster->header = cloud_filtered->header;
                    pcl::toROSMsg(*cloud_cluster, out);
                    c_pub.publish(out);
                    j++;
                }
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
    ros::spin ();

}
