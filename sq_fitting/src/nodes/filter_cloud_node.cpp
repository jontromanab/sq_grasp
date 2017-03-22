#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include  <pcl/point_types.h>
#include <pcl/filters/filter.h>


ros::Publisher pub;

double ws_max_x, ws_min_x, ws_max_y, ws_min_y, ws_max_z, ws_min_z;

void setLimits(const std::vector<double>& ws_limits)
{
  ws_min_x = ws_limits[0];
  ws_max_x = ws_limits[1];
  ws_min_y = ws_limits[2];
  ws_max_y = ws_limits[3];
  ws_min_z = ws_limits[4];
  ws_max_z = ws_limits[5];
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *cloud_);
  for(size_t i=0;i<cloud_->size();++i)
  {
    if(cloud_->points[i].x > ws_min_x && cloud_->points[i].x < ws_max_x)
    {
      if(cloud_->points[i].y > ws_min_y && cloud_->points[i].y < ws_max_y)
      {
        if(cloud_->points[i].z > ws_min_z && cloud_->points[i].z < ws_max_z)
        {
          cloud_filtered->points.push_back(cloud_->points[i]);
        }
       }
    }
  }
  cloud_filtered->height =1;
  cloud_filtered->header.frame_id = cloud_->header.frame_id;
  cloud_filtered->width = cloud_filtered->points.size();

  //Removing Nans
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr wo_nan_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_filtered, *wo_nan_cloud, indices);


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*wo_nan_cloud, output);
  pub.publish(output);
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "filter_cloud");
  ros::NodeHandle nh_("~");
  std::string cloud_topic_;
  std::vector<double> workspace_limits_;
  nh_.getParam("workspace",workspace_limits_);
  nh_.getParam("cloud_topic", cloud_topic_);
  setLimits(workspace_limits_);

  ros::Subscriber sub = nh_.subscribe(cloud_topic_, 1, cloud_cb);
  pub = nh_.advertise<sensor_msgs::PointCloud2> ("cloud_filtered",1);
  ros::spin();
}
