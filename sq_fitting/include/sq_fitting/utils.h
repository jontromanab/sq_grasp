#ifndef UTILS_H
#define UTILS_H
#include<sq_fitting/sq.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;



void sq_clampParameters(double& e1_clamped, double& e2_clamped);

double sq_function(const PointT& point, const sq_fitting::sq& param);

double sq_function(const double &x, const double &y, const double &z, const double &a, const double &b, const double &c, const double &e1,
                   const double &e2);

double sq_function_scale_weighting(const PointT& point, const sq_fitting::sq &param);

double sq_error(const pcl::PointCloud<PointT>::Ptr cloud, const sq_fitting::sq& param);

void sq_create_transform(const geometry_msgs::Pose& pose, Eigen::Affine3f& transform);

double sq_normPoint(const PointT& point);

void euler2Quaternion (const double roll, const double pitch, const double yaw, Eigen::Quaterniond& q);

void create_transformation_matrix(const double tx, const double ty, const double tz, const double ax, const double ay, const double az, Eigen::Affine3d &trns_mat);

void create_rotation_matrix(const double ax, const double ay, const double az, Eigen::Affine3d &rot_matrix);

void getParamFromPose(const geometry_msgs::Pose& pose, double& tx, double& ty, double& tz, double& ax, double& ay, double& az );

void Quaternion2Euler(const geometry_msgs::Pose& pose, double& ax, double& ay, double& az);



#endif // UTILS_H
