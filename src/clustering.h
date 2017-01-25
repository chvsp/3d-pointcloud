#include "includes.h"
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

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

bool
enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  else
    return (false);
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


template <class PointType>
void EuclideanClustering(pcl::PointCloud<PointType>::Ptr cloud_in, std::vector<pcl::PointIndices> cluster_indices, float cluster_tolerance = 0.02, int minClusterSize = 100,int maxClusterSize = 25000)
{
pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
tree->setInputCloud (cloud_in);
pcl::EuclideanClusterExtraction<PointType> ec;
ec.setClusterTolerance (cluster_tolerance); // 2cm
ec.setMinClusterSize (minClusterSize);
ec.setMaxClusterSize (maxClusterSize);
ec.setSearchMethod (tree);
ec.setInputCloud (cloud_in);
ec.extract (cluster_indices);
}


template <class PointType>
void ConditionalEuclideanClustering(pcl::PointCloud<PointType>::Ptr cloud_in, std::vector<pcl::PointIndices> cluster_indices, float cluster_tolerance = 0.02, int minClusterSize = 100,int maxClusterSize = 25000, int mode = 1)
{
   pcl::IndicesClustersPtr small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
   pcl::ConditionalEuclideanClustering<PointType> cec (true);
   cec.setInputCloud (cloud_in);
   switch(mode)
   {
     case 1:
     cec.setConditionFunction  (&enforceIntensitySimilarity);
     break;
     case 2:
     cec.setConditionFunction (&customRegionGrowing);
     break;
     case 3:
     cec.setConditionFunction (&enforceCurvatureOrIntensitySimilarity);
     break;
     default:
     cec.setConditionFunction  (&enforceIntensitySimilarity);
     break;

   }
   float cluster_tolerance = 0.02, int minClusterSize = 100,int maxClusterSize = 25000
   cec.setClusterTolerance (cluster_tolerance);
   cec.setMinClusterSize (minClusterSize);
   cec.setMaxClusterSize (maxClusterSize);
   cec.segment (*clusters);
   cec.getRemovedClusters (small_clusters, large_clusters);
}
