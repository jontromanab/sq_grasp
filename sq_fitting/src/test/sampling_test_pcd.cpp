#include<iostream>
#include<sq_fitting/sampling.h>
#include<sq_fitting/sq.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


int main(int argc, char *argv[])
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined(new pcl::PointCloud<pcl::PointXYZRGB>()); // ::Ptr or cloud_combined is a shared_ptr
/*
 => Use raw pointers when you do not want to have any ownership attached to the pointer. This job can also often be done with references. Raw pointers can also be used in
some low level code (such as for implementing smart pointers, or implementing containers).
=> Use unique_ptr or scope_ptr when you want unique ownership of the object. This is the most useful option, and should be used in most cases. Unique ownership
can also be expressed by simply creating an object directly, rather than using a pointer (this is even better than using a unique_ptr, if it can be done).
=> Use shared_ptr or intrusive_ptr when you want shared ownership of the pointer. This can be confusing and inefficient, and is often not a good option.
Shared ownership can be useful in some complex designs, but should be avoided in general, because it leads to code which is hard to understand
 */

  sq_fitting::sq super;
  super.a1 = 0.012925;
  super.a2 = 0.0064596;
  super.a3 = 0.116491;
  super.e1 = 1.307882;
  super.e2 = 1.268144;
  geometry_msgs::Pose pose; // from sq.h: pose is a template so however we define the pose here (geometry_msgs::Pose) it is going to replace the template
  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 1.0;
  pose.orientation.w = 1.0;
  super.pose = pose;


  sq_fitting::sq super2;
  super2.a1 = 0.0101381;
  super2.a2 = 0.0389216;
  super2.a3 = 0.0760457;
  super2.e1 = 0.1;
  super2.e2 = 0.29569;
  geometry_msgs::Pose pose2;
  pose2.position.x = 1.0;
  pose2.position.y = 0.25;
  pose2.position.z = 1.0;
  pose2.orientation.w = 1.0;
  super2.pose = pose2;

  SuperquadricSampling *sam = new SuperquadricSampling(super);
  sam->sample_pilu_fisher(); // Pilu-Fisher is a method to do the sampling
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  sam->getCloud(cloud); // getCloud assigns the local PointCloud made by pilu_fisher method inside sam pointer object to the input PointCloud which is cloud2 here
  std::cout<<"size of the first sampled cloud: "<<cloud->points.size()<<std::endl;

  SuperquadricSampling *sam2 = new SuperquadricSampling(super2);
  sam2->sample_pilu_fisher();
  pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
  sam2->getCloud(cloud2);
  std::cout<<"size of the second sampled cloud: "<<cloud2->points.size()<<std::endl;


  *cloud_combined = *cloud2; // both are shared_ptr. This line means the value that cloud2 is pointing to is going to be assigned to the value that cloud_combined is pointing to
  //*cloud_combined+= *cloud2;


  std::cout<<"Size of combined superquadrics cloud: "<<cloud_combined->points.size()<<std::endl;
  pcl::io::savePCDFileASCII ("superquadrics.pcd", *cloud_combined);
  std::cerr << "Saved " << cloud_combined->points.size () << " data points to superquadrics.pcd." << std::endl;

  return 0;
}
