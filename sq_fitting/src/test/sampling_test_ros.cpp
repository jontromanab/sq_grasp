#include <ros/ros.h>
#include<sq_fitting/sampling.h>
#include<sq_fitting/sq.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "sampling_test");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("superquadrics", 1);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("superquadrics2", 1);

  sq_fitting::sq super;
  //coke can 0.0276175,0.0342516,0.0561649,0.389707,1.0314
  /*super.a1 = 0.02;
  super.a2 = 0.07;
  super.a3 = 0.05;
  super.e1 = 0.1;
  super.e2 = 0.1;*/
  super.a1 = 0.0276175;
  super.a2 = 0.0342516;
  super.a3 = 0.0561649;
  super.e1 = 0.389707;
  super.e2 = 1.0314;
  geometry_msgs::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;
  super.pose = pose;


  sq_fitting::sq super2;
  //box 0.0439586,0.0519484,0.0480955,0.602776,0.60603
  //Sphere
  /*super2.a1 = 0.05;
  super2.a2 = 0.05;
  super2.a3 = 0.05;
  super2.e1 = 1.00333;
  super2.e2 = 0.952999;*/
  super2.a1 = 0.0439586;
  super2.a2 = 0.0519484;
  super2.a3 = 0.0180955;
  super2.e1 = 0.2;
  super2.e2 = 0.2;
  geometry_msgs::Pose pose2;
  pose2.position.x = 1.0;
  pose2.position.y = 0.25;
  pose2.position.z = 1.0;
  pose2.orientation.w = 1.0;
  super2.pose = pose2;


  Sampling *sam = new Sampling(super);
  sam->sample_pilu_fisher();
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  sam->getCloud(cloud);
  std::cout<<"size of the sampled cloud: "<<cloud->points.size()<<std::endl;
  sensor_msgs::PointCloud2 ros_cloud;
  sam->getCloud(ros_cloud);
  ros_cloud.header.frame_id = "/world";
  ros_cloud.header.stamp = ros::Time::now();


  Sampling *sam2 = new Sampling(super2);
  pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
  sam2->sample_pilu_fisher();
  sam2->getCloud(cloud2);
  std::cout<<"size of the second sampled cloud: "<<cloud2->points.size()<<std::endl;
  sensor_msgs::PointCloud2 ros_cloud2;
  sam2->getCloud(ros_cloud2);
  ros_cloud2.header.frame_id = "/world";
  ros_cloud2.header.stamp = ros::Time::now();



  ros::Rate loop_rate(1);
  while(nh.ok())
  {
    pub.publish(ros_cloud);
    pub2.publish(ros_cloud2);
    ros::spinOnce();
    loop_rate.sleep();
  }


}
