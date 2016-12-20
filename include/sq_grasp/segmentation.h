#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <iostream>
#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


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

struct Object
{
  pcl::PointCloud<PointT> object_cloud;
  int label;
};

class Segmentation
{
public:
 //Parameters for segmentation. sacsegmentation -> convexhull -> supervoxel ->lccp
  struct Parameters
  {
    //general parameters
    double zmin_; //min distance from the table plane to be considered as object cloud
    double zmax_; //max distatance from the table plabe to be considered as object cloud
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


  /*Constructor
  param: ros node
  param: cloud_topic the ROS topic that contains the input point cloud
  param: Set of parameters for segmentation
  */
  Segmentation(ros::NodeHandle& node, const std::string& cloud_topic, const Parameters& params);

  //~Segmentation();

  //Segments the objects on the table
  bool segment();
  void detectObjectsOnTable(pcl::PointCloud<PointT>::Ptr cloud, double zmin, double zmax, bool filter_input_cloud);
  void segmentClusterIntoObjects(pcl::PointCloud<PointT>::Ptr cloud);
  void detectedObjectToCloud();
  bool initialized;
  //Clouds

  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr table_plane_cloud; //table plane cloud
  pcl::PointCloud<PointT> convexHull; //convex hull of the table
  pcl::PointCloud<PointT> clusters; //object clusters
  pcl::ModelCoefficients plane_coefficients; //coefficients of table plane
  //pcl::PointCloud<PointT> segmented_objects_cloud;

  //Supervoxel lccp stuff
  std::vector<Object> detected_objects;
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud;
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud;
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud;

  sensor_msgs::PointCloud2 plane_cloud;
  sensor_msgs::PointCloud2 object_cloud;
  sensor_msgs::PointCloud2 convex_hull_cloud;
  sensor_msgs::PointCloud2 segmented_objects_msg;
  sensor_msgs::PointCloud2 input_msg_;// just keeping track of the input msg



  ros::Subscriber cloud_sub_; //Subscriber for point cloud topic

  ros::Publisher cloud_pub_; //Publishing the point clouds
  ros::Publisher table_pub_; //Publishing the segmented table. NOt needed. Just for testing
  ros::Publisher convex_hull_pub_;//Publishing the convex hull of the table. Not needed. Just for testing
  ros::Publisher  segmented_objects_pub_;

private:
  //Callback function for the ROS topic that contains the input cloud
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);

  //Publishes the objects and  table
  void publish_clouds();

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

  //Setting parameters
  void set_zmin(double zmin_in);
  void set_zmax(double zmax_in);


  //Getting parameters
  double get_zmin();
  double get_zmax();
  void print_parameters();



};

#endif // SEGMENTATION_H
