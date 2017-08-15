#include<sq_fitting/segmentationfloor.h>


Segmentation::Segmentation():
  cloud_(),indices_(new pcl::PointIndices),
  horizontal_tolerance_degrees_(10),
  margin_above_surface_(0.005),
  cluster_distance_(0.01),
  min_cluster_size_(10),
max_cluster_size_(10000)
{
  this->objects_.resize(0);
  this->poses_.resize(0);
}

void Segmentation::set_input_cloud(CloudPtr cloud)
{
  cloud_ = cloud;
}
void Segmentation::set_indices(pcl::PointIndices::Ptr indices)
{
  indices_ = indices;
}
void Segmentation::set_cluster_distance(double cluster_distance)
{
  cluster_distance_=cluster_distance;
}
void Segmentation::set_horizontal_tolerance_degrees(double deg)
{
  horizontal_tolerance_degrees_ = deg;
}
void Segmentation::set_margin_above_surface(double margin)
{
  margin_above_surface_ = margin;
}
void Segmentation::set_max_cluster_size(int max_cluster_size)
{
  max_cluster_size_ = max_cluster_size;
}
void Segmentation::set_min_cluster_size(int min_cluster_size)
{
  min_cluster_size_=min_cluster_size;
}
void Segmentation::get_poses(std::vector<geometry_msgs::Pose> &poses)
{
  poses = poses_;
}
void Segmentation::get_segmented_clouds(std::vector<CloudPtr> &cloud)
{
  cloud=objects_;
}



