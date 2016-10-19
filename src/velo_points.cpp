#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


using namespace std;

void chatterCallback(const sensor_msgs::PointCloud2& input)
{
	// pcl::visualization::CloudViewer::CloudViewer("Hello");
	pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ip(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*ip);

  pcl::PointCloud<pcl::PointXYZ> p = *ip;

	cout << p.points[1].x << " " << p.points[1].y << " " << p.points[1].z << endl; 
	return;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "taker");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/velodyne_points", 100, chatterCallback);

	ros::spin();

	return 0;
}

