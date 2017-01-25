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

#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<velodyne_pointcloud::PointXYZIR> temp;
int i = 0, it =0;
pcl::PointCloud<pcl::PointXYZI> p;
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

void chatterCallback(const sensor_msgs::PointCloud2& input)
{

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ip(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2,*ip);

  viewer.showCloud (ip);
  if(i<1){
    pcl::PCDWriter writerfi;
    std::stringstream ss;
    ss << "cloud_filtered_p" << i << ".pcd";
    writerfi.write<pcl::PointXYZI> (ss.str (), *ip, false); //*
    pcl::io::savePCDFileASCII (ss.str(), *ip);
  i++;}
  for(int x=0;x<100000;x++)
  {
    int a = 1+2;
  }
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
