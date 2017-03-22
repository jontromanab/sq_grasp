#ifndef SEGMENTATION_H
#define SEGMENTATION_H

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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr CloudPtr;

struct Object
{
  PointCloud object_clouds;
  int label;
};

class Segmentation{
public:
  struct Parameters
  {
    double zmin;
    double zmax;
    int th_points;
    //superVoxel parameters
    bool disable_transform;
    double voxel_resolution;
    double seed_resolution;
    double color_importance;
    double spatial_importance;
    double normal_importance;
    //LCCP segmentation parameters
    double concavity_tolerance_threshold;
    double smoothness_threshold;
    int min_segment_size;
    bool use_extended_convexity;
    bool use_sanity_criterion;
  };
  Segmentation(const CloudPtr& input_cloud, const Parameters& param);
  bool segment();
  bool initialized;
  void getTablecloud(CloudPtr &table_cloud);
  void getObjectsCloud(CloudPtr &object_cloud);
  void getObjectsOnTable(CloudPtr &objects_on_table);
  void getObjects(std::vector<CloudPtr>& objects);

private:

  void detectObjectsOnTable(CloudPtr cloud, double zmin, double zmax, bool filter_input_cloud_);
  void segmentClusterIntoObjects(CloudPtr clusters);
  void detectedObjectsToCloud();



  CloudPtr cloud_;
  CloudPtr table_plane_cloud_;
  CloudPtr segmented_objects_cloud_;
  CloudPtr objects_on_table_;
  PointCloud convexHull;
  pcl::ModelCoefficients plane_coefficients_;
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud;

  std::vector<Object> detected_objects_;
  std::vector<CloudPtr> objects_; //vector of segmented clouds

  //parameters
  double zmin_;
  double zmax_;
  int th_points_;

  //Supervoxel parameters
  bool disable_transform_;
  double voxel_resolution_;
  double seed_resolution_;
  double color_importance_;
  double spatial_importance_;
  double normal_importance_;

  //LCCP segmentation parameters
  double concavity_tolerance_threshold_;
  double smoothness_threshold_;
  int min_segment_size_;
  bool use_extended_convexity_;
  bool use_sanity_criterion_;

};



#endif // SEGMENTATION_H
