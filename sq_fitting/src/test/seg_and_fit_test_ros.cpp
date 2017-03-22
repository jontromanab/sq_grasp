#include<ros/ros.h>
#include<sq_fitting/segmentation.h>
#include<sq_fitting/fitting.h>
#include<sq_fitting/sampling.h>
#include<sq_fitting/sq.h>


#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher sq_pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  Segmentation::Parameters params;
  params.zmin = 0.03;
  params.zmax = 2.0;
  params.th_points = 200;
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

  pcl::PointCloud<PointT>::Ptr input_cloud_pcl(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr sq_cloud_pcl(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *input_cloud_pcl);

  //Segmentation
  Segmentation* seg = new Segmentation(input_cloud_pcl, params);
  seg->segment();


  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  objects.resize(0);
  seg->getObjects(objects);
  //std::cout<<"Detected objects: "<<objects.size()<<std::endl;


  //Fitting
  for(int i=0;i<objects.size();++i)
  {
    pcl::PointCloud<PointT>::Ptr filtered_cloud_pcl(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (objects[i]);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered_cloud_pcl);

    SuperquadricFitting* sq_fit = new SuperquadricFitting(filtered_cloud_pcl);
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
    Sampling *sam = new Sampling(min_param);
    sam->sample_pilu_fisher();
    pcl::PointCloud<PointT>::Ptr sq_cloud(new pcl::PointCloud<PointT>);
    sam->getCloud(sq_cloud);
    for(size_t k=0;k<sq_cloud->points.size();++k)
    {
      PointT tmp;
      tmp.x = sq_cloud->points.at(k).x;
      tmp.y = sq_cloud->points.at(k).y;
      tmp.z = sq_cloud->points.at(k).z;
      tmp.r = sq_cloud->points.at(k).r;
      tmp.g = sq_cloud->points.at(k).g;
      tmp.b = sq_cloud->points.at(k).b;
      sq_cloud_pcl->points.push_back(tmp);
     }
   }

  sq_cloud_pcl->height = 1;
  sq_cloud_pcl->width = sq_cloud_pcl->points.size();
  sq_cloud_pcl->is_dense = true;


  sensor_msgs::PointCloud2 sq_cloud_ros;
  pcl::toROSMsg(*sq_cloud_pcl, sq_cloud_ros);
  //sq_cloud_ros.header.frame_id = "camera_ir_optical_frame";//for kinect1
  sq_cloud_ros.header.frame_id = "kinect2_ir_optical_frame";//for kinect2
  sq_cloud_ros.header.stamp = ros::Time::now();
  sq_pub.publish(sq_cloud_ros);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seg_and_fit_test");
  ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe("/kinect2/sd/points",1,cloud_cb);
  ros::Subscriber sub = nh.subscribe("/cloud_filter_node/cloud_filtered",1,cloud_cb);
  sq_pub = nh.advertise<sensor_msgs::PointCloud2> ("superquadrics",10);

  ros::spin();

}
