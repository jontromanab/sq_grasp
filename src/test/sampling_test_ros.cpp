#include <ros/ros.h>
#include<sq_fitting/sampling.h>
#include<sq_fitting/sq.h>
#include<geometry_msgs/PoseArray.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "sampling_test");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("superquadrics", 1);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("superquadrics2", 1);
  ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("superquadrics3", 1);
  ros::Publisher pub4 = nh.advertise<geometry_msgs::PoseArray>("poses",1);


  sq_fitting::sq super3;
  super3.a1 = 0.01;
  super3.a2 = 0.08;
  super3.a3 = 0.05;
  super3.e1 = 0.1;
  super3.e2 = 0.1;
  geometry_msgs::Pose pose3;
  pose3.position.x = 0.612908;
  pose3.position.y = 0.128006;
  pose3.position.z = 0.747510;
  pose3.orientation.w = 1.0;
  super3.pose = pose3;

  geometry_msgs::PoseArray poseArr;
  poseArr.poses.push_back(pose3);
  poseArr.header.frame_id  = "/base_link";
  poseArr.header.stamp = ros::Time::now();

  Sampling *sam3 = new Sampling(super3);
  sam3->sample_pilu_fisher();
  pcl::PointCloud<PointT>::Ptr cloud3(new pcl::PointCloud<PointT>);
  sam3->getCloud(cloud3);
  std::cout<<"size of the sampled cloud: "<<cloud3->points.size()<<std::endl;
  sensor_msgs::PointCloud2 ros_cloud3;
  sam3->getCloud(ros_cloud3);
  ros_cloud3.header.frame_id = "/base_link";
  ros_cloud3.header.stamp = ros::Time::now();

/*  Sampling *sam = new Sampling(super);
  sam->sample();
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
  ros_cloud2.header.frame_id = "/base_link";
  ros_cloud2.header.stamp = ros::Time::now();*/



  ros::Rate loop_rate(1);
  while(nh.ok())
  {
    //pub.publish(ros_cloud);
    //pub2.publish(ros_cloud2);
    pub3.publish(ros_cloud3);
    pub4.publish(poseArr);
    ros::spinOnce();
    loop_rate.sleep();
  }


}
