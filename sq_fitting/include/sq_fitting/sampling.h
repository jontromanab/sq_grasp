#ifndef SAMPLING_H
#define SAMPLING_H

#include<iostream>
#include<ros/ros.h>
#include<sq_fitting/sq.h>
#include<sq_fitting/utils.h>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;

class Sampling
{
public:
  Sampling(const sq_fitting::sq& sq_params);
  void sample();
  void sample_pilu_fisher();
  void sample_superEllipse(const double a1, const double a2, const double e, const int N, pcl::PointCloud<PointT>::Ptr& cloud);

  void getCloud(pcl::PointCloud<PointT>::Ptr& cloud);
  void getCloud(sensor_msgs::PointCloud2& cloud_ros);



private:
  pcl::PointCloud<PointT>::Ptr cloud_;
  sensor_msgs::PointCloud2 cloud_ros_;
  sq_fitting::sq params_;
  float r_, g_, b_;
  void transformCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud, pcl::PointCloud<PointT>::Ptr& output_cloud);

};

#endif // SAMPLING_H
