#ifndef SAMPLING_H
#define SAMPLING_H

#include <iostream>
#include <ros/ros.h>
#include <sq_fitting/sq.h>
#include <sq_fitting/utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>

//typedef
typedef pcl::PointXYZRGB PointT;

/**
 * @brief Sample superquadrics based on provided parameters
 */
class SuperquadricSampling
{
public:
  /**
   * @brief Constructor
   * @param sq_params ros msg for superquadrics
   */
  SuperquadricSampling(const sq_fitting::sq& sq_params);

  /**
   * @brief Sampling by superquadric equation
   */
  void sample();

  /**
   * @brief Sampling by Pilu Fisher method for superellipsoids
   */
  void sample_pilu_fisher();

  /**
   * @brief Sampling by Pilu Fisher method for superteroids
   */
  void sample_pilu_fisher_st();

  /**
   * @brief obtain cloud
   * @param cloud
   */
  void getCloud(pcl::PointCloud<PointT>::Ptr& cloud);

  /**
   * @brief obtain cloud ros
   * @param cloud_ros
   */
  void getCloud(sensor_msgs::PointCloud2& cloud_ros);



private:
  pcl::PointCloud<PointT>::Ptr cloud_;
  sensor_msgs::PointCloud2 cloud_ros_;
  sq_fitting::sq params_;
  float r_, g_, b_;

  /**
   * @brief transform Cloud by SQ pose transformation
   * @param input_cloud
   * @param output_cloud
   */
  void transformCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud, pcl::PointCloud<PointT>::Ptr& output_cloud);

};

#endif // SAMPLING_H
