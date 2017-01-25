#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
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

// #include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

// #include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// #include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>
using namespace pcl;
using namespace std;
void rg_seg(pcl::PointCloud<PointXYZ>::Ptr cloud );

void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)

{
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_innerhfilter (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outerhfilter (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointIndices::Ptr indices_rem (new pcl::PointIndices);



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr ptree (new pcl::search::KdTree<pcl::PointXYZ>);

pcl::VoxelGrid<pcl::PointXYZ> vg;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // vg.setInputCloud (cloud);
  // vg.setLeafSize (0.1f, 0.1f, 0.1f);
  // vg.filter (*cloud_filtered);
  cloud_filtered = cloud;
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.2);
  int i=0;
  int nr_points = (int) cloud_filtered->points.size ();



  while (i < 2)//cloud_filtered->points.size () > 0.4 * nr_points)  ///----------------------------------------------------------------------- planar segmentation
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
    i++;
    std::cout << "PointCloud representing the planar component" << i << ": " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;


  }
  i = 0;
  pcl::PCDWriter writerfi;
  std::stringstream ss;
  ss << "cloud_filtered_x"  << ".pcd";
  writerfi.write<pcl::PointXYZ> (ss.str (), *cloud_plane, false); //*
  pcl::io::savePCDFileASCII (ss.str(), *cloud_plane);

  rg_seg(cloud_filtered);
}

void rg_seg(pcl::PointCloud<PointXYZ>::Ptr cloud )
{

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  pcl::PCDWriter writer;
  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0,j=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

   // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   // viewer.showCloud (cloud_cluster);
   // getchar();
   // while (!viewer.wasStopped ())
   // {
   // }
   // getchar();
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "rg_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    pcl::io::savePCDFileASCII (ss.str(), *cloud_cluster);

    j++;
  }
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

}
















pcl::PointCloud<velodyne_pointcloud::PointXYZIR> temp;
int i = 0, it =0;
pcl::PointCloud<pcl::PointXYZ> p;
// pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

void chatterCallback(const sensor_msgs::PointCloud2& input)
{

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ip(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*ip);
  // viewer.showCloud (ip);
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

  ros::spin();

  return 0;
}
