#include<iostream>
#include<sq_fitting/segmentation.h>
#include<sq_fitting/fitting.h>
#include<sq_fitting/sampling.h>
#include<sq_fitting/sq.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char *argv[])
{

  if(argc !=2)
    return (0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sq_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  if(pcl::io::loadPCDFile(argv[1], *cloud)==-1)
     return (-1);

  std::cout<<"size of input cloud: "<<cloud->size()<<std::endl;
  std::unique_ptr<lccp_segmentation> seg(new lccp_segmentation);
  seg->init(*cloud);
  seg->segment();


  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  objects = seg->get_segmented_objects_simple();
  std::cout<<"Detected objects: "<<objects.size()<<std::endl;


  for(int i=0;i<objects.size();++i)
  {
    pcl::PointCloud<PointT>::Ptr filtered_cloud_pcl(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (objects[i]);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered_cloud_pcl);

    SuperquadricFitting* sq_fit = new SuperquadricFitting(filtered_cloud_pcl);
    sq_fit->set_pose_est_method("pca");
    //sq_fit->set_pose_est_method = iteration;
    sq_fitting::sq min_param;
    double min_fit;
    sq_fit->fit();
    sq_fit->getMinParams(min_param);
    sq_fit->getMinError(min_fit);

    std::cout<<"Minimum error["<<i<<"] is : "<<min_fit<<std::endl;
    std::cout<<"Minimum parameters["<<i<<"] is: "<<"a1:"<<min_param.a1<<"  a2:"
            <<min_param.a2<<" a3:"<<min_param.a3<<" e1:"<<min_param.e1<<" e2:"<<min_param.e2<<" position:"
            <<min_param.pose.position.x<<" "<<min_param.pose.position.y<<min_param.pose.position.z<<" orientation:"
            <<min_param.pose.orientation.x<<" "<<min_param.pose.orientation.y<<" "<<min_param.pose.orientation.z<<" "
            <<min_param.pose.orientation.w<<std::endl;

    //Sampling
    SuperquadricSampling *sam = new SuperquadricSampling(min_param);
    sam->sample_pilu_fisher();
    pcl::PointCloud<PointT>::Ptr s_cloud(new pcl::PointCloud<PointT>);
    sam->getCloud(s_cloud);
    for(size_t k=0;k<s_cloud->points.size();++k)
    {
      PointT tmp;
      tmp.x = s_cloud->points.at(k).x;
      tmp.y = s_cloud->points.at(k).y;
      tmp.z = s_cloud->points.at(k).z;
      tmp.r = s_cloud->points.at(k).r;
      tmp.g = s_cloud->points.at(k).g;
      tmp.b = s_cloud->points.at(k).b;
      sq_cloud->points.push_back(tmp);
     }
    delete sq_fit;
    delete sam;
   }

  sq_cloud->height = 1;
  sq_cloud->width = sq_cloud->points.size();
  sq_cloud->is_dense = true;


  pcl::io::savePCDFileASCII ("objects_superquadrics.pcd", *sq_cloud);
  std::cerr << "Saved " << sq_cloud->points.size () << " data points to objects_superquadrics.pcd." << std::endl;
  return 0;
}
