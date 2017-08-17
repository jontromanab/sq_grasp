#include<ros/ros.h>
#include<sq_fitting/segmentation.h>
#include<pcl/filters/crop_box.h>
#include <pcl/surface/convex_hull.h>


ros::Publisher table_pub, object_on_table_pub, object_pub, filter_pub, hull_pub,projected_pub;
float  boundary[6] = {-0.5, 0.2,  -3, 3, 0.7, 1.5};

void filterCloud(const CloudPtr& cloud, CloudPtr& filtered_cloud)
{
  CloudPtr cloud_nan(new PointCloud);

//  pcl::CropBox<PointT> crop;
//  crop.setInputCloud(cloud);
//  Eigen::Vector4f min;
//  min<<boundary[0] ,boundary[2],boundary[4],1;
//  Eigen::Vector4f max;
//  max<<boundary[1] ,boundary[3],boundary[5],1;
//  crop.setMin(min);
//  crop.setMax(max);
//  crop.filter(*cloud_nan);
  std::vector<int> indices;
  for(size_t  i=0;i<cloud->points.size();++i)
  {
     if(cloud->points[i].x > boundary[0]  && cloud->points[i].x < boundary[1] )
     {
       if(cloud->points[i].y > boundary[2]  && cloud->points[i].y < boundary[3] )
       {
         if(cloud->points[i].z > boundary[4]  && cloud->points[i].z < boundary[5] )
         {
           filtered_cloud->points.push_back(cloud->points[i]);
         }
       }
     }
  }

  //pcl::removeNaNFromPointCloud(*cloud_nan, *filtered_cloud, indices);
}

void getConvexHull(const CloudPtr& cloud, CloudPtr& hull_cloud)
{
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (cloud);
  chull.reconstruct (*hull_cloud);
}

void projectToZ(const CloudPtr& cloud_in, CloudPtr& output_cloud)
{

  double val_x = cloud_in->points.at(0).x;
  double val_y = cloud_in->points.at(0).y;
  double val_z = cloud_in->points.at(0).z;
  double x_max = val_x, x_min = val_x, y_max = val_y, y_min = val_y, z_max = val_z, z_min = val_z;
  for(int i=0;i<cloud_in->points.size();++i)
  {
    if (cloud_in->points.at(i).x>=x_max)
      x_max = cloud_in->points.at(i).x;
    if (cloud_in->points.at(i).x<=x_min)
      x_min = cloud_in->points.at(i).x;
    if (cloud_in->points.at(i).y>=y_max)
      y_max = cloud_in->points.at(i).y;
    if (cloud_in->points.at(i).y<=y_min)
      y_min = cloud_in->points.at(i).y;
    if (cloud_in->points.at(i).z>=z_max)
      z_max = cloud_in->points.at(i).z;
    if (cloud_in->points.at(i).z<=z_min)
      z_min = cloud_in->points.at(i).z;
  }



  for(size_t  i=0;i<cloud_in->points.size();++i)
  {
    pcl::PointXYZRGB p;
    p.x = x_min;
    p.y = cloud_in->points[i].y;
    p.z = cloud_in->points[i].z;
    p.rgb = cloud_in->points[i].rgb;
    output_cloud->points.push_back(p);
  }
  output_cloud->header.frame_id = "kinect2_link";
  output_cloud->height = 1;
  output_cloud->width = output_cloud->points.size();
}

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
  pcl::PointCloud<PointT>::Ptr filtered_cloud_pcl(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *input_cloud_pcl);
  filterCloud(input_cloud_pcl, filtered_cloud_pcl);


  std::cout<<"Size of input cloud: "<<input_cloud_pcl->points.size()<<std::endl;
  std::cout<<"Size of filtered cloud: "<<filtered_cloud_pcl->points.size()<<std::endl;

  Segmentation* seg = new Segmentation(filtered_cloud_pcl, params);
  seg->segment();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  seg->getTablecloud(table_cloud);
  std::cout<<"Size of table cloud: "<<table_cloud->points.size()<<std::endl;
  sensor_msgs::PointCloud2 table_cloud_ros;
  pcl::toROSMsg(*table_cloud, table_cloud_ros);
  table_cloud_ros.header.frame_id = "kinect2_link";
  table_cloud_ros.header.stamp = ros::Time::now();
  table_pub.publish(table_cloud_ros);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_on_table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  seg->getObjectsOnTable(object_on_table_cloud);
  std::cout<<"Size of cloud on table: "<<object_on_table_cloud->points.size()<<std::endl;
  sensor_msgs::PointCloud2 object_cloud_ros;
  pcl::toROSMsg(*object_on_table_cloud, object_cloud_ros);
  object_cloud_ros.header.frame_id = "kinect2_link";
  object_cloud_ros.header.stamp = ros::Time::now();
  object_on_table_pub.publish(object_cloud_ros);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  seg->getObjectsCloud(object_cloud);
  std::cout<<"Size of segmented objects cloud: "<<object_cloud->points.size()<<std::endl;
  sensor_msgs::PointCloud2 objects_cloud_ros;
  pcl::toROSMsg(*object_cloud, objects_cloud_ros);
  objects_cloud_ros.header.frame_id = "kinect2_link";
  objects_cloud_ros.header.stamp = ros::Time::now();
  object_pub.publish(objects_cloud_ros);

  pcl::PointCloud<PointT>::Ptr projected_cloud_pcl(new pcl::PointCloud<PointT>);
  projectToZ(object_cloud, projected_cloud_pcl );
  sensor_msgs::PointCloud2 projected_cloud_ros;
  pcl::toROSMsg(*projected_cloud_pcl, projected_cloud_ros);
  projected_cloud_ros.header.frame_id = "kinect2_link";
  projected_cloud_ros.header.stamp = ros::Time::now();
  projected_pub.publish(projected_cloud_ros);


  pcl::PointCloud<PointT>::Ptr hull_cloud_pcl(new pcl::PointCloud<PointT>);
  getConvexHull(projected_cloud_pcl , hull_cloud_pcl );
  sensor_msgs::PointCloud2 hull_cloud_ros;
  pcl::toROSMsg(*hull_cloud_pcl, hull_cloud_ros);
  hull_cloud_ros.header.frame_id = "kinect2_link";
  hull_cloud_ros.header.stamp = ros::Time::now();
  hull_pub.publish(hull_cloud_ros);



  sensor_msgs::PointCloud2 filtered_cloud_ros;
  pcl::toROSMsg(*filtered_cloud_pcl, filtered_cloud_ros);
  filtered_cloud_ros.header.frame_id = "kinect2_link";
  filtered_cloud_ros.header.stamp = ros::Time::now();
  filter_pub.publish(filtered_cloud_ros);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  seg->getObjects(objects);
  std::cout<<"Detected objects: "<<objects.size()<<std::endl;


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_test");
  ros::NodeHandle nh;

  //after cloud_filter_node is running
  //ros::Subscriber sub = nh.subscribe("/cloud_filter_node/cloud_filtered",1,cloud_cb);
  ros::Subscriber sub = nh.subscribe("kinect2/sd/points",1,cloud_cb);
  table_pub = nh.advertise<sensor_msgs::PointCloud2> ("table_cloud",10);
  object_on_table_pub = nh.advertise<sensor_msgs::PointCloud2> ("object",10);
  object_pub = nh.advertise<sensor_msgs::PointCloud2> ("objects",10);
  filter_pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",10);
  hull_pub = nh.advertise<sensor_msgs::PointCloud2> ("convex_hull",10);
  projected_pub = nh.advertise<sensor_msgs::PointCloud2> ("projected_cloud",10);
  ros::spin();

}
