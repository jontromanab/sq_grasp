#ifndef SEGMENTATIONFLOOR_H
#define SEGMENTATIONFLOOR_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>

#include<pcl/filters/extract_indices.h>
#include<pcl/surface/concave_hull.h>
#include<pcl/segmentation/extract_polygonal_prism_data.h>
#include<pcl/segmentation/supervoxel_clustering.h>
#include<pcl/segmentation/lccp_segmentation.h>

#include<sq_fitting/utils.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr CloudPtr;

class Segmentation
{
public:
  Segmentation();

  bool segment();


  void set_input_cloud(CloudPtr cloud);
  void set_indices(pcl::PointIndices::Ptr indices);
  void set_horizontal_tolerance_degrees(double deg);
  void set_cluster_distance(double cluster_distance);
  void set_min_cluster_size(int min_cluster_size);
  void set_max_cluster_size(int max_cluster_size);
  void set_margin_above_surface(double margin);

  void get_segmented_clouds(std::vector<CloudPtr>& cloud);
  void get_poses(std::vector<geometry_msgs::Pose>& poses);


private:
  std::vector<CloudPtr> objects_;//collection of objects
  std::vector<geometry_msgs::Pose> poses_;//collection of poses of the objects

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::PointIndicesPtr indices_;

  double horizontal_tolerance_degrees_;
  double margin_above_surface_;
  double cluster_distance_;
  int min_cluster_size_;
int max_cluster_size_;
};


#endif // SEGMENTATIONFLOOR_H
