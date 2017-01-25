#include "point_types.h"
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <typeinfo>
// #include <libcluster/distributions.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


using namespace std;

#define iteration 20
#define innerRadius 10
#define MinNeighborsInRadius 10




void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)

{

 // Create the filtering object: downsample the dataset using a leaf size of 1cm
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::VoxelGrid<pcl::PointXYZ> vg;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
 // vg.setInputCloud (cloud);
 // vg.setLeafSize (0.01f, 0.01f, 0.01f);
 // vg.filter (*cloud_filtered);
 std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
 *cloud_filtered = *cloud;
 // Create the segmentation object for the planar model and set all the parameters
 pcl::SACSegmentation<pcl::PointXYZ> seg;
 pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
 pcl::PCDWriter writer;
 seg.setOptimizeCoefficients (true);
 seg.setModelType (pcl::SACMODEL_PLANE);
 seg.setMethodType (pcl::SAC_RANSAC);
 seg.setMaxIterations (100);
 seg.setDistanceThreshold (0.4);
 int k=1;
 int i=0, nr_points = (int) cloud_filtered->points.size ();
 while (k--)//cloud_filtered->points.size () > 0.3 * nr_points)
 {
   // Segment the largest planar component from the remaining cloud
   seg.setInputCloud (cloud_filtered);
   seg.segment (*inliers, *coefficients);
   if (inliers->indices.size () == 0)
   {
     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
     break;
   }

   // Extract the planar inliers from the input cloud
   pcl::ExtractIndices<pcl::PointXYZ> extract;
   extract.setInputCloud (cloud_filtered);
   extract.setIndices (inliers);
   extract.setNegative (false);

   // Get the points associated with the planar surface
   extract.filter (*cloud_plane);
   std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  //  pcl::PCDWriter writerfi;
  //  std::stringstream ss;
  //  ss << "cloud_p" << k << ".pcd";
  //  writerfi.write<pcl::PointXYZ> (ss.str (), *cloud_plane, false); //*
  //  pcl::io::savePCDFileASCII (ss.str(), *cloud_plane);
  //  Remove the planar inliers, extract the rest
   extract.setNegative (true);
   extract.filter (*cloud_f);
   *cloud_filtered = *cloud_f;
 }

 // Creating the KdTree object for the search method of the extraction
 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 tree->setInputCloud (cloud_filtered);

 std::vector<pcl::PointIndices> cluster_indices;
 pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
 ec.setClusterTolerance (0.5); // 2cm
 ec.setMinClusterSize (30);
 ec.setMaxClusterSize (25000);
 ec.setSearchMethod (tree);
 ec.setInputCloud (cloud_filtered);
 ec.extract (cluster_indices);

 int j = 0;
 for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
 {
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
     cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
   cloud_cluster->width = cloud_cluster->points.size ();
   cloud_cluster->height = 1;
   cloud_cluster->is_dense = true;

   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
   std::stringstream ss;
   ss << "cloud_cluster_" << j << ".pcd";
   writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
   j++;
 }
cout << "End\n";
}








pcl::PointCloud<velodyne_pointcloud::PointXYZIR> temp;
int i = 0, it =0;
pcl::PointCloud<pcl::PointXYZ> p;

void chatterCallback(const sensor_msgs::PointCloud2& input)
{

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ip(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*ip);
  if(i<1){
  clustering(ip);
  i++;}
  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud");

  ros::NodeHandle n;

  ros::NodeHandle m;
  ros::Subscriber sub = n.subscribe("/velodyne_points", 1000000, chatterCallback);

  // ros::Publisher chatter_pub = m.advertise< pcl::PointCloud<pcl::PointXYZ> >("pcl_pointcloud", 100);



  // std::cout << typeid(p.makeShared()).name() << endl;
  // chatter_pub.publish(p.makeShared());

  ros::spin();

  return 0;
}
