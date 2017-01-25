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
pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_planes (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointIndices::Ptr indices_rem (new pcl::PointIndices);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_innerhfilter (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr ptree (new pcl::search::KdTree<pcl::PointXYZ>);
pcl::VoxelGrid<pcl::PointXYZ> vg;


// pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilteri (true); // inner radius filter
// rorfilteri.setInputCloud (cloud);
// rorfilteri.setRadiusSearch (innerRadius);
// rorfilteri.setMinNeighborsInRadius (MinNeighborsInRadius);
// rorfilteri.setNegative (true);
// rorfilteri.filter (*cloud_innerhfilter);
std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  //
  // vg.setInputCloud (cloud);
  // vg.setLeafSize (0.1f, 0.1f, 0.1f);
  // vg.filter (*cloud_filtered);
  // cloud_filtered = cloud;

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.6);
  int i=0;
  int nr_points = (int) cloud_filtered->points.size ();



  while ( i < 3)//cloud_filtered->points.size () > 0.5 * nr_points)// )  ///------------------3000 or 2000----------------------------------------------------- planar segmentation
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    i++;
    std::cout << "PointCloud representing the planar component" << i << ": " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
     *cloud_planes += *cloud_plane ;
    //  nr_points = (int) cloud_plane->points.size ();

    // pcl::PCDWriter writerfi;
    // std::stringstream ss;
    // ss << "cloud_filtered_p" << i << ".pcd";
    // writerfi.write<pcl::PointXYZ> (ss.str (), *cloud_plane, false); //*
    // pcl::io::savePCDFileASCII (ss.str(), *cloud_plane);


  }
  i = 0;

  //
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud_planes);
  // pass.setFilterFieldName ("intensity");
  // pass.setFilterLimits (90, 100);
  // //pass.setFilterLimitsNegative (true);
  //
  // pass.filter (*cloud_filtered);
  // pcl::PCDWriter writerfi;
  // std::stringstream ss;
  // ss << "cloud_filteredone" << i << ".pcd";
  // writerfi.write<pcl::PointXYZ> (ss.str (), *cloud_planes, false); //*
  // std::cout << "PointCloud representing the planar component"  << ": " << cloud_planes->points.size () << " data points." << std::endl;
  // pcl::io::savePCDFileASCII (ss.str(), *cloud_planes);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr test (new pcl::PointCloud<pcl::PointXYZ> ());
  //
  // test->width  = cloud_filtered->width;
  // test->height = 1;
  // test->points.resize (test->width * test->height);
  //
  // for (size_t i = 0; i < test->points.size (); ++i)
  // {
  //   test->points[i].x = cloud_filtered->points[i].x;
  //   test->points[i].y = cloud_filtered->points[i].y;
  //   test->points[i].z = (cloud_filtered->points[i].intensity) / 12  ;
  // }




  // pass.setInputCloud (cloud_planes);
  // pass.setFilterFieldName ("intensity");
  // pass.setFilterLimits (20, 40);
  // //pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_filtered);
  // ss << "cloud_filteredtwo" << i << ".pcd";
  // writerfi.write<pcl::PointXYZ> (ss.str (), *cloud_filtered, false); //*
  // pcl::io::savePCDFileASCII (ss.str(), *cloud_filtered);





    // Creating the KdTree objec5t for the search method of the extraction
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_innerhfilter (new pcl::PointCloud<pcl::PointXYZ>);
    //  cloud_innerhfilter = cloud_filtered;
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfiltero (false); // outer radius filter
    // rorfiltero.setInputCloud (cloud_innerhfilter);
    // rorfiltero.setRadiusSearch (outerRadius);
    // rorfiltero.setMinNeighborsInRadius (MinNeighborsInRadius);
    // rorfiltero.setNegative (false);
    // rorfiltero.filter (*cloud_filtered);


  float thresh = 0.4;
  int j = 0;
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  pcl::PCDWriter writer;
  std::stringstream ssd;
  ssd << "cloud_fone"  << ".pcd";
  writer.write<pcl::PointXYZ> (ssd.str (), *cloud_planes, false); //*
  pcl::io::savePCDFileASCII (ssd.str(), *cloud_planes);
  // Extract the planar inliers from the input cloud
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers);
  //
  // // Get the points associated with the planar surface
  // extract.filter (*cloud_plane);
  // i++;
  // std::cout << "PointCloud representing the planar component" << i << ": " << cloud_plane->points.size () << " data points." << std::endl;
  //
  // //     // Extract the planar inliers from the input cloud
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers);
  //
  // // Get the points associated with the planar surface
  // extract.filter (*cloud_plane);
  // i++;
  // std::cout << "PointCloud representing the planar component" << i << ": " << cloud_plane->points.size () << " data points." << std::endl;
  //
  // // Remove the planar inliers, extract the rest
  // extract.setNegative (true);
  // extract.filter (*cloud_f);
  // *cloud_filtered = *cloud_f;
  //  *cloud_planes += *cloud_plane ;    // Extract the planar inliers from the input cloud
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud (cloud_filtered);
  // extract.setIndices (inliers);
  //
  // // Get the points associated with the planar surface
  // extract.filter (*cloud_plane);
  // i++;
  // std::cout << "PointCloud representing the planar component" << i << ": " << cloud_plane->points.size () << " data points." << std::endl;
  //
  // // Remove the planar inliers, extract the rest
  // extract.setNegative (true);
  // extract.filter (*cloud_f);
  // *cloud_filtered = *cloud_f;
  //  *cloud_planes += *cloud_plane ;
// while(1 /*thresh > 0.5*///)                     //-------------------------------------------------------------------------------- Euclidean Clustering


  // thresh = 1.5;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (thresh); // 2cm = 0.02
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (200000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract(cluster_indices);

  // cout << bool(cluster_indices.begin() == cluster_indices.end());
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    // cout << "Enter\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    } //*
    // cout << "Cluster extracted\n";
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    if (cloud_cluster->points.size () > 0)
    {
    std::stringstream ss;
    ss << "cloud_cluster_" << thresh << "_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    pcl::io::savePCDFileASCII (ss.str(), *cloud_cluster);
    j++;
    }

    cout << cloud_filtered->size() << endl;
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
