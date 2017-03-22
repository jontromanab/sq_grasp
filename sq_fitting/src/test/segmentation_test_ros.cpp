#include<ros/ros.h>
#include<sq_fitting/segmentation.h>

ros::Publisher table_pub, object_on_table_pub, object_pub;

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
  pcl::fromROSMsg(*input, *input_cloud_pcl);

  Segmentation* seg = new Segmentation(input_cloud_pcl, params);
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
  ros::spin();

}
