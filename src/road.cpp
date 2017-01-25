#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
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


typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

bool
enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  // if (fabs (point_a.intensity - point_b.intensity) > 50.0f )
  //   return (false);
   if( point_a.intensity < 20 && point_a.intensity < 20)
  return (true);
  else
  return(false);
}

bool
enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
    return (true);
  return (false);
}

bool
customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
  if (squared_distance < 10000)
  {
    if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  }
  else
  {
    if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
      return (true);
  }
  return (false);
}

int
main (int argc, char** argv)
{
  // Data containers used
  pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);
  pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
  // pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
  pcl::console::TicToc tt;
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZINormal>);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr  cloud_planes (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr  cloud_plane (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointIndices::Ptr indices_rem (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZINormal>);

  // Load the input point cloud
  std::cerr << "Loading...\n", tt.tic ();
  pcl::io::loadPCDFile ("roadnew.pcd", *cloud_in);
  std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->points.size () << " points\n";

  // Downsample the cloud using a Voxel Grid class
  // std::cerr << "Downsampling...\n", tt.tic ();
  // pcl::VoxelGrid<PointTypeIO> vg;
  // vg.setInputCloud (cloud_in);
  // vg.setLeafSize (80.0, 80.0, 80.0);
  // vg.setDownsampleAllData (true);
  // vg.filter (*cloud_out);
  // std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->points.size () << " points\n";

    // Set up a Normal Estimation class and merge data in cloud_with_normals


  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloud_in, *cloud_with_normals);
  pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch (1);
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";



  //Planar Segmentation

  pcl::SACSegmentation<pcl::PointXYZINormal> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (1);
  int i=0;
  int nr_points = (int) cloud_filtered->points.size ();



  while ( i < 1)//cloud_filtered->points.size () > 0.5 * nr_points)// )  ///------------------3000 or 2000----------------------------------------------------- planar segmentation
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_with_normals);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZINormal> extract;
    extract.setInputCloud (cloud_with_normals);
    extract.setIndices (inliers);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    i++;
    std::cout << "PointCloud representing the planar component" << i << ": " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_with_normals = *cloud_f;

    // cloud_planes only contains the planes
      *cloud_planes += *cloud_plane ;
    //  nr_points = (int) cloud_plane->points.size ();
      // pcl::PCDWriter writerfi;
      // std::stringstream ss;
      // ss << "cloud_filtered_p" << i << ".pcd";
      // writerfi.write<pcl::PointXYZINormal> (ss.str (), *cloud_plane, false); //*
      // pcl::io::savePCDFileASCII (ss.str(), *cloud_plane);


  }

  // Set up a Conditional Euclidean Clustering class
  std::cerr << "Segmenting to clusters...\n", tt.tic ();
  pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
  cec.setInputCloud (cloud_planes);
  cec.setConditionFunction (&enforceIntensitySimilarity);
  cec.setClusterTolerance (0.05);
  cec.setMinClusterSize (100);
  cec.setMaxClusterSize (cloud_planes->points.size ());
  cec.segment (clusters);
  // cec.getRemovedClusters (small_clusters, large_clusters);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  int clustercounter = 0;

  int k = 0;
  // for (int i = 0; i < clusters->size (); ++i)
  // {
  //
  //   for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
  //   {
  //   cloudclus->points[k].x = cloud_planes->points[(*clusters)[i].indices[j]].x;
  //   cloudclus->points[k].y = cloud_planes->points[(*clusters)[i].indices[j]].y;
  //   cloudclus->points[k].z = cloud_planes->points[(*clusters)[i].indices[j]].z;
  //   cloudclus->points[k].intensity = cloud_planes->points[(*clusters)[i].indices[j]].intensity;
  //   k++;
  //   }
  // }
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr  road (new pcl::PointCloud<pcl::PointXYZINormal>);
  for (std::vector<pcl::PointIndices>::const_iterator it = (clusters).begin (); it != (clusters).end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr  cloudclus (new pcl::PointCloud<pcl::PointXYZINormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloudclus->points.push_back (cloud_planes->points[*pit]);
    }
    k++;
    cloudclus->width = cloudclus->points.size ();
    cloudclus->height = 1;
    cloudclus->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloudclus->points.size () << " data points." << std::endl;

    if (cloudclus->points.size () > 0)
    {
      pcl::PCDWriter writer;
      std::stringstream ss1;
    *road += *cloudclus;
    ss1 << "cloud_cluster_"<< k  << "_" << ".pcd";
    writer.write<pcl::PointXYZINormal> (ss1.str (), *cloudclus, false); //*
    pcl::io::savePCDFileASCII (ss1.str(), *cloudclus);
    }



  }
  pcl::SACSegmentation<pcl::PointXYZINormal> segroad;
  pcl::PointIndices::Ptr inliersroad (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficientsroad (new pcl::ModelCoefficients);
  segroad.setOptimizeCoefficients (true);
  segroad.setModelType (pcl::SACMODEL_PLANE);
  segroad.setMethodType (pcl::SAC_RANSAC);
  segroad.setMaxIterations (1000);
  segroad.setDistanceThreshold (0.00001);
  i=0;
  std::cout << "PointCloud  " << road->points.size () << " data points." << std::endl;
  pcl::PCDWriter writerfi;
  std::stringstream ss;
  ss << "cloud_bumper" << ".pcd";
  writerfi.write<pcl::PointXYZINormal> (ss.str (), *road, false); //*
  pcl::io::savePCDFileASCII (ss.str(), *road);
  while ( i < 1)//cloud_filtered->points.size () > 0.5 * nr_points)// )  ///------------------3000 or 2000----------------------------------------------------- planar segmentation
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (road);
    seg.segment (*inliersroad, *coefficientsroad);
    if (inliersroad->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZINormal> extract;
    extract.setInputCloud (road);
    extract.setIndices (inliersroad);

    // Get the points associated with the planar surface
    i++;
    std::cout << "PointCloud representing the planar component" << i << ": " << road->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *road = *cloud_f;

    // cloud_planes only contains the planes

  }




  // Save the output point cloud



  return (0);
}
