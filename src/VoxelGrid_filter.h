#include "includes.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
// Create the filtering object
template <class PointType>
void VoxelGridFilter(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_filtered, float leaf_size = 0.1f)
{
pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
vg.setInputCloud (cloud_in);
vg.setLeafSize (leaf_size, leaf_size, leaf_size);
vg.filter (*cloud_filtered);
}
