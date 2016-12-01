/*
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;
using namespace pcl::console;
using namespace pcl::visualization;

class SimpleHDLViewer
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    SimpleHDLViewer (pcl::Grabber& grabber,
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> &handler) :
        cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL HDL Cloud")),
        grabber_ (grabber),
        handler_ (handler)
    {
    }

    void cloud_callback (const CloudConstPtr& cloud)
    {
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void run ()
    {
      cloud_viewer_->addCoordinateSystem (3.0);
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);

      boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
          &SimpleHDLViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (
          cloud_cb);

      grabber_.start ();

      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr cloud;

        // See if we can get a cloud
        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          handler_.setInputCloud (cloud);
          if (!cloud_viewer_->updatePointCloud (cloud, handler_, "HDL"))
            cloud_viewer_->addPointCloud (cloud, handler_, "HDL");

          cloud_viewer_->spinOnce ();
        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();

        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();

      cloud_connection.disconnect ();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;

    CloudConstPtr cloud_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
};

int main (int argc, char ** argv)
{
  std::string hdlCalibration, pcapFile;

  parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  parse_argument (argc, argv, "-pcapFile", pcapFile);

  pcl::Grabber grabber(hdlCalibration, pcapFile);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler("intensity");

  SimpleHDLViewer v(grabber, color_handler);
  v.run();
  return (0);
}

*/

/*

pcl::PointCloud<velodyne_pointcloud::PointXYZIR> p;

void chatterCallback(const sensor_msgs::PointCloud2& input)
{
  // pcl::visualization::CloudViewer::CloudViewer("Hello");
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input,pcl_pc2);
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr ip(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  pcl::fromPCLPointCloud2(pcl_pc2,*ip);

   p = *ip;

  std::cout << p.points[1].x << " " << p.points[1].y << " " << p.points[1].z << " " << p.points[1].ring << endl; 

  // ros::Publisher chatter_pub = n.advertise< pcl::PointCloud<pcl::PointXYZ> >("pcl_pointcloud", 100);

  // chatter_pub.publish(p.makeShared());
  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud");
  
  ros::NodeHandle n;

  ros::NodeHandle m;
  ros::Subscriber sub = n.subscribe("/velodyne_points", 100, chatterCallback);

  ros::Publisher chatter_pub = m.advertise< pcl::PointCloud<velodyne_pointcloud::PointXYZIR> >("pcl_pointcloud", 100);

  

  std::cout << typeid(p.makeShared()).name() << endl;
  chatter_pub.publish(p.makeShared());

  ros::spin();

  return 0;
}

*/

/**
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 *
 
*/

/*
 STL
#include <iostream>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>


int 
main (int, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PCDWriter writer;
	
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_ptr) == -1)
  {
    std::cout<<"Couldn't read the file "<<argv[1]<<std::endl;
    return (-1);
  }
  std::cout << "Loaded pcd file " << argv[1] << " with " << cloud_ptr->points.size () << std::endl;

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_ptr);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod (tree_n);
  ne.setRadiusSearch (0.03);
  ne.compute (*cloud_normals);
  std::cout << "Estimated the normals" << std::endl;

  // Creating the kdtree object for the search method of the extraction
  boost::shared_ptr<pcl::KdTree<pcl::PointXYZ> > tree_ec  (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  tree_ec->setInputCloud (cloud_ptr);
  
  // Extracting Euclidean clusters using cloud and its normals
  std::vector<int> indices;
  std::vector<pcl::PointIndices> cluster_indices;
  const float tolerance = 0.5f; // 50cm tolerance in (x, y, z) coordinate system
  const double eps_angle = 5 * (M_PI / 180.0); // 5degree tolerance in normals
  const unsigned int min_cluster_size = 50;
 
  pcl::extractEuclideanClusters (*cloud_ptr, *cloud_normals, tolerance, tree_ec, cluster_indices, eps_angle, min_cluster_size);

  std::cout << "No of clusters formed are " << cluster_indices.size () << std::endl;

  // Saving the clusters in seperate pcd files
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]); 
    cloud_cluster->width = static_cast<uint32_t> (cloud_cluster->points.size ());
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster using xyzn: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "./cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    j++;
  }

  return (0);
}

*/

/*
----------------------CURRENT RUNNING----------------------------------

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
#include <libcluster/distributions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/visualization/cloud_viewer.h>

using namespace std;

void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
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
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
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

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  cout <<"before\n";
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (25000);
  cout <<"1\n";
  ec.setSearchMethod (tree);
  cout <<"2\n";
  ec.setInputCloud (cloud_filtered);
  cout <<"3\n";
  ec.extract (cluster_indices);
  cout <<"hello";
  int j = 0;
  cout << bool(cluster_indices.begin() == cluster_indices.end());
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
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
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    pcl::io::savePCDFileASCII (ss.str(), *cloud);
  }
}

pcl::PointCloud<pcl::PointXYZ> p;
int i = 0;

void chatterCallback(const sensor_msgs::PointCloud2& input)
{

  // pcl::visualization::CloudViewer::CloudViewer("Hello");
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ip(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*ip);

   p = *ip;

  // std::cout << p.points[1].x << " " << p.points[1].y << " " << p.points[1].z << " " << p.points[1].ring << endl; 

  // ros::Publisher chatter_pub = n.advertise< pcl::PointCloud<pcl::PointXYZ> >("pcl_pointcloud", 100);

  // chatter_pub.publish(p.makeShared());
   if(i==0)
    clustering(ip);

   ++i;
  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud");
  
  ros::NodeHandle n;

  ros::NodeHandle m;
  ros::Subscriber sub = n.subscribe("/velodyne_points", 1000000, chatterCallback);

  // ros::Publisher chatter_pub = m.advertise< pcl::PointCloud<velodyne_pointcloud::PointXYZIR> >("pcl_pointcloud", 100);

  

  // std::cout << typeid(p.makeShared()).name() << endl;
  // chatter_pub.publish(p.makeShared());

  ros::spin();

  return 0;
}

*/


