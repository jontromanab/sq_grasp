#include<iostream>
#include<sq_fitting/segmentation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char *argv[])
{

  if(argc !=2)
    return (0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  if(pcl::io::loadPCDFile(argv[1], *cloud)==-1)
     return (-1);

  std::cout<<"size of input cloud: "<<cloud->size()<<std::endl;
  Segmentation::Parameters params;
  params.zmin = 0.03;
  params.zmax = 2.0;
  params.th_points = 50;
  params.voxel_resolution = 0.0075f;
  params.seed_resolution = 0.015f;
  params.color_importance = 0.0f;
  params.spatial_importance = 1.0f;
  params.normal_importance = 4.0f;
  params.use_extended_convexity = false;
  params.use_sanity_criterion = true;
  params.concavity_tolerance_threshold = 10;
  params.smoothness_threshold = 0.1f;
  params.min_segment_size = 3;

  Segmentation* seg = new Segmentation(cloud, params);
  seg->segment();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  seg->getTablecloud(table_cloud);
  std::cout<<"Size of table cloud: "<<table_cloud->points.size()<<std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_on_table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  seg->getObjectsOnTable(object_on_table_cloud);
  std::cout<<"Size of cloud on table: "<<object_on_table_cloud->points.size()<<std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  seg->getObjectsCloud(object_cloud);
  std::cout<<"Size of segmented objects cloud: "<<object_cloud->points.size()<<std::endl;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  seg->getObjects(objects);
  std::cout<<"Detected objects: "<<objects.size()<<std::endl;


  pcl::io::savePCDFileASCII ("segmented_objects.pcd", *object_cloud);
  std::cerr << "Saved " << object_cloud->points.size () << " data points to segmented_objects.pcd." << std::endl;
  return 0;
}
