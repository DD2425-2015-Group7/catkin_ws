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
class Plane_Extraction{

    ros::NodeHandle n;

    ros::Publisher c_pub;
    ros::Publisher obj_loc_pub;
    ros::Publisher bb_publish;
    ros::Publisher i_pub;
    //ros::Subscriber c_sub;
    //ros::Subscriber b_box;

public:

    void run(){
        using namespace message_filters;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::Image> Policy;

        c_pub = n.advertise<sensor_msgs::PointCloud2> ("/clustered_object_cloud", 1);
        obj_loc_pub = n.advertise<geometry_msgs::PolygonStamped>("/object_point/uncompared",20);
        bb_publish =  n.advertise<plane_extraction::BoundingBox_FloatArray>("/pointCloud_detector/bounding_boxes", 50); //detection::BoundingBoxArray>
        i_pub = n.advertise<sensor_msgs::Image> ("/object_detection/Image",20);

        //std::cerr << "in the initialization "<< std::endl;

        //std::cerr << "After Register" << std::endl;

       // c_sub = n.subscribe ("/camera/depth_registered/points", 1, &Plane_Extraction::cloud_callback, this);
        message_filters::Subscriber<sensor_msgs::PointCloud2> Cloud(n, "/camera/depth_registered/points", 30);
        message_filters::Subscriber<sensor_msgs::Image> image(n, "/camera/rgb/image_rect_color",30);
        Synchronizer<Policy> sync(Policy(50),Cloud, image);

        //std::cerr << "before callback " << std::endl;
        sync.registerCallback(boost::bind(&Plane_Extraction::cloud_callback,this, _1, _2));
        ros::Rate loop_rate(5);

        ros::spin ();
        loop_rate.sleep();
    }

    std::vector<int> indexToCoordinates(int index, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
    {
        int x = 1, y = 1;
        std::vector<int> coord;
        while (index >= (cloud->width))
        {
            index -= (cloud->width);
            x++;
        }
        y = index + 1;
        coord.push_back(x);
        coord.push_back(y);
        std::cout << "The pixel is " << x << "," << y << std::endl;
        return coord;
    }
    void PassFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,int xmin,int xmax,int ymin,int ymax){
        pcl::PassThrough<pcl::PointXYZRGB> filterx;
        pcl::PassThrough<pcl::PointXYZRGB> filtery;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr smaller_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        filterx.setFilterFieldName("x");
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud = *input;
        //std::cerr << "min X: " << xmin << ", max X: " << xmax << std::endl;
        //std::cerr << "min y: " <<ymin<< ", max y: " << ymax<< std::endl;
       /* float xmin1 = cloud(xmin, ymin).x; // (270, 190) for centered bounding box...
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
        */
        /*float xmin1 = -2;
        float xmax1 =
        std::cerr << "min Xe: " << xmin1 << ", max Xe: " << xmax1 << std::endl;
        std::cerr << "min ye: " <<ymin1<< ", max ye: " << ymax1<< std::endl;

        filterx.setFilterLimits(xmin1, xmax1);
        filterx.setFilterLimitsNegative (false);
        filterx.filter(*smaller_cloud);*/
        float ymin1 = -2;
        float ymax1 = 0.20;
        filtery.setFilterFieldName("y");
        filtery.setFilterLimitsNegative (false);
        filtery.setInputCloud(input);

        filtery.setFilterLimits(ymin1,ymax1);

        filtery.filter(*output);


    }
    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg, const sensor_msgs::Image::ConstPtr& image_msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered_blob(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB> cloud_blob_test;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr init_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(*msg,*init_cloud);
        pcl::fromROSMsg(*msg,cloud_blob_test);
        sensor_msgs::Image i_to_pub;

       // std::cerr << "msg_b->bounding_boxes.size() "<< msg_b->bounding_boxes.size() << std::endl;

        double x_width;
        double y_width;
        float x_0, y_0;
        float x_1, y_1;
        int isObject;

        //PassFilter(init_cloud, cloud_blob, 0, 192,60, 580);
        cloud_blob = init_cloud;

        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
        // Create the filtering object: downsample the dataset using a leaf size of 1cm when 0.01f on all. When using it for our objects 1cm is too big and 1 mm is too small for the amount of particles that we have.
        //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_blob);
        sor.setLeafSize(0.005f, 0.005f, 0.1f);
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
       // std::cerr << "Between IFS" << std::endl;
        if(cloud_filtered->width*cloud_filtered->height != 0 ){
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud(cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setClusterTolerance (0.01); // 10cm
            ec.setMinClusterSize (1200);
            ec.setMaxClusterSize (15000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            //find objects and put them in a new separate point cloud. only takes the objects that consist of 900-3000 particles

            if(cluster_indices.begin() == cluster_indices.end()){
                //std::cerr << "samma" << std::endl;
            }else{
                int j = 0;
                plane_extraction::BoundingBox_FloatArray bbox_array_msg;
                geometry_msgs::PolygonStamped point_array;
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                {
                    //std::cerr << "==========================New Cluster====================================" << std::endl;
                    //uncomment this and comment the previous declaration if you only want the "newest" object to be found
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //

                    }
                    Eigen::Vector4f centroid;
                    geometry_msgs::Point p;
                    pcl::compute3DCentroid(*cloud_cluster, centroid);
                    p.x = (double) centroid[0];
                    p.y = (double) centroid[1];
                    p.z = (double) centroid[2];

                    if(centroid[2] < 1 ){
                        // build the condition
                        int rMax = 50;
                        int rMin = 0;
                        int gMax = 50;
                        int gMin = 0;
                        int bMax = 50;
                        int bMin = 0;
                        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
                        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
                        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
                        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
                        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
                        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
                        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

                        // build the filter
                        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem (color_cond);
                        condrem.setInputCloud (cloud_cluster);
                        condrem.setKeepOrganized(true);

                        // apply filter
                        condrem.filter (*cloud);

                        /*std::cout << "The XYZ coordinates of the centroid are: ("
                                  << centroid[0] << ", "
                                  << centroid[1] << ", "
                                  << centroid[2] << ")." << std::endl;*/
                        std::cout << "PointCloud representing the Cluster: " << cloud->points.size () << " data points." << std::endl;



                        x_width = cloud->width + 20;
                        y_width = cloud->height + 20;

                        //Determine the dimension of the bounding box ===========================
                        std::cerr << "before if"<< std::endl;
                        if(cloud->points.size() > 100){


                            Eigen::Vector4f min_pt, max_pt;
                            pcl::getMinMax3D (*cloud_filtered,it->indices, min_pt, max_pt);
                            double x_0_3D = min_pt[0];
                            double y_0_3D = min_pt[1];
                            double x_1_3D = max_pt[0];
                            double y_1_3D = max_pt[1];


                            /*for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pit = cloud_filtered->points.begin(); pit < cloud_filtered->points.end (); ++pit){
                        if(pit->x == x_0_3D){
                            if(pit->y == y_0_3D){
                                std::cerr << "det är samma " << std::endl;
                            }

                        }


                    }*/

                            //std::cerr << "the points bb: "<< "("<<x_0_3D<< ", " << y_0_3D <<", " << x_1_3D << ", " << y_1_3D << ")" << std::endl;
                            plane_extraction::BoundingBox_Float bbox_msg;
                           // cloud = *cloud_cluster;
                            //std::cerr << "the points bb: "<< "("<<x_0_3D<< ", " << y_0_3D <<", " << x_1_3D << ", " << y_1_3D << ")" << std::endl;

                            // =====================================================================
                            std::vector<int> temp0;
                            std::vector<int> temp1;

                            x_0 = float(x_0_3D);
                            x_1 = float(x_1_3D);
                            y_0 = float(y_0_3D);
                            y_1 = float(y_1_3D);
std::cerr << "after if"<< std::endl;
                            bbox_msg.x0 = x_0;
                            bbox_msg.y0 = y_0;
                            bbox_msg.x1 = x_1;
                            bbox_msg.y1 = y_1;
                            //std::cerr << "the points bb: "<< "("<<x_0<< ", " << y_0<<", " << x_1 << ", " << y_1 << ")" << std::endl;

                           // cloud->width = cloud->points.size ();

                            if(cloud->points.size() > 5000){ //TBD
                                isObject = 0;
                                // std::cerr << "skräp vid: "<< "(" << centroid[0] << ", " << centroid[1] <<", " << centroid[2] << ")."<< std::endl;
                            }else{
                                isObject = 1;
                            }
                            bbox_msg.prob = float(isObject);
                            bbox_array_msg.bounding_boxes.push_back(bbox_msg);
                            cloud_cluster->height = 1;
                            cloud_cluster->is_dense = true;


                            sensor_msgs::PointCloud2 out;
                            geometry_msgs::Point32 pkt;

                            pkt.x = centroid[2]; //in the robot coordinate, needs to be done in TF, maybe array to fit with bounding box??
                            pkt.y = centroid[0];
                            pkt.z = centroid[1];

                            pkt.x = pkt.x*cos(atan2(pkt.z,pkt.x)); //fixed with tf later?

                            cloud->header = cloud_filtered->header;
                            pcl::toROSMsg(*cloud, out);


                            //what to publish: bounding box : bb message,< publishes in the same msg. done
                            //centroid : geometry_msgs/Point.msg done    |
                            // and isObject : std_msgs/Bool.msg. --------|
                            c_pub.publish(out);
                            //obj_loc_pub.publish(pkt);
                            point_array.polygon.points.push_back(pkt);
                        }
                    }
                    j++;
                }
                obj_loc_pub.publish(point_array);
                //i_to_pub.header = point_array.header;
                i_pub.publish(i_to_pub);
                bb_publish.publish(bbox_array_msg);
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
