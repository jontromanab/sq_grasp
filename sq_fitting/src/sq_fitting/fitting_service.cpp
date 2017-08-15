#include "ros/ros.h"
#include<sq_fitting/get_sq_param.h>
#include<sq_fitting/fitting.h>
#include<sq_fitting/segmentation.h>

bool add(sq_fitting::get_sq_param::Request  &req,
         sq_fitting::get_sq_param::Response &res)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   std::cout<<"We ar eworking"<<std::endl;
   pcl::fromROSMsg(req.point_cloud, *cloud);

   //pcl::fromROSMsg(req.point_cloud,cloud);

   SuperquadricFitting* sq_fit = new SuperquadricFitting(cloud);
   sq_fitting::sq min_param;
   sq_fit->fit();
   sq_fit->getMinParams(min_param);
   res.super_param = min_param;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fitting_service");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("get_sq_param", add);
  ROS_INFO("Ready to get parameter from point cloud");
  ros::spin();

  return 0;
}
