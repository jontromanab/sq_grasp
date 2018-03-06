#include<iostream>
#include<sq_fitting/sampling.h>
#include<sq_fitting/sq.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<memory>
typedef pcl::PointCloud<PointT>::Ptr pointCloudPtr;


pointCloudPtr create_sq_cloud(const double e1, const double e2, const double x,
                                                   const double y, const double z){
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  sq_fitting::sq super;
  super.a1 = 0.1;
  super.a2 = 0.1;
  super.a3 = 0.1;
  //super.a4 = 2.0;
  super.e1 = e1;
  super.e2 = e2;
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  super.pose = pose;
  std::unique_ptr<SuperquadricSampling> samp(new SuperquadricSampling(super));
  samp->sample_pilu_fisher();
  samp->getCloud(cloud);
  return cloud;
}


int main(int argc, char *argv[])
{
  pointCloudPtr cloud_combined(new pcl::PointCloud<PointT>());
    pointCloudPtr cloud(new pcl::PointCloud<PointT>());
    for (double i=0.0;i<=2.5;i+=0.5){
        for (double j=0.0;j<=2.5;j+=0.5){
          if(i==0.0)
            i+=0.1;
          if(j==0.0)
            j+=0.1;

          cloud = create_sq_cloud(i,j, j, (2.5-i), 0.0); // for a1, a2, a3 = 0.1, a4 = 2.0
          //cloud = create_st_cloud(i,j, j/10.0, (2.5-i)/10.0, 0.0); // for a1, a2, a3 = 0.01, a4 = 0.2
          std::cout<<"Creating cloud# "<<"e1: "<<i<<" e2: "<<j<<" x: "<<j*2.0<<" y: "<<(2.5-i)*2.0<<std::endl;// for a1, a2, a3 = 0.1, a4 = 2.0
          //std::cout<<"Creating cloud# "<<"e1: "<<i<<" e2: "<<j<<" x: "<< j/10.0<<" y: "<<(2.5-i)/10.0<<std::endl; // for a1, a2, a3 = 0.01, a4 = 0.2
          if(i==0.1)
            i-=0.1;
          if(j==0.1)
            j-=0.1;
          *cloud_combined+=*cloud;
        }
      }

    pcl::io::savePCDFileASCII ("big_superquadrics.pcd", *cloud_combined);
    std::cerr << "Saved " << cloud_combined->points.size () << " data points to big_superquadrics.pcd." << std::endl;
    return 0;

}
