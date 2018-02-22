#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sq_fitting/segmentation.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <chrono>



class SegmentationClient
{
public:
  SegmentationClient(ros::NodeHandle* nodeHandle, std::string cloud_topic, std::string segmentation_service);
  void cloudCallback(const sensor_msgs::PointCloud2& msg);
  void publishClouds();
private:
  ros::NodeHandle nh_;
  std::string cloud_topic_ ;
  std::string segmentation_service_;
  ros::Subscriber sub_;
  ros::Publisher table_cloud_pub_;
  ros::Publisher object_cloud_pub_;
  sensor_msgs::PointCloud2 table_cloud_ros_;
  sensor_msgs::PointCloud2 object_cloud_ros_;
};

SegmentationClient::SegmentationClient(ros::NodeHandle *nodeHandle, std::string cloud_topic, std::string segmentation_service)
  :nh_(*nodeHandle),cloud_topic_(cloud_topic),segmentation_service_(segmentation_service){
  sub_ = nh_.subscribe(cloud_topic_, 1, &SegmentationClient::cloudCallback, this);
  table_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table",10);
  object_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("objects",10);
}

void SegmentationClient::cloudCallback(const sensor_msgs::PointCloud2& msg){
  ros::ServiceClient client = nh_.serviceClient<sq_fitting::segment_object>(segmentation_service_);
  sq_fitting::segment_object srv;
  srv.request.input_cloud = msg;
  if(client.call(srv)){
    table_cloud_ros_ = srv.response.plane_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> segmented_objects_cloud;
    for(int i=0;i<srv.response.object_cloud.size();++i){
      pcl::PointCloud<pcl::PointXYZRGB> tmp;
      pcl::fromROSMsg(srv.response.object_cloud[i], tmp);
      float r = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      float g = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      float b = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      for(int j=0;j<tmp.points.size();++j){
        pcl::PointXYZRGB temp_point;
        temp_point.x = tmp.points.at(j).x;
        temp_point.y = tmp.points.at(j).y;
        temp_point.z = tmp.points.at(j).z;
        temp_point.r = r * 255;
        temp_point.g = g * 255;
        temp_point.b = b * 255;
        segmented_objects_cloud.points.push_back(temp_point);
      }
    }
    segmented_objects_cloud.width = segmented_objects_cloud.points.size();
    segmented_objects_cloud.height =1;
    segmented_objects_cloud.is_dense = true;
    pcl::toROSMsg(segmented_objects_cloud, object_cloud_ros_);
    object_cloud_ros_.header.seq = 1;
    object_cloud_ros_.header.frame_id = msg.header.frame_id;
    object_cloud_ros_.header.stamp = ros::Time::now();
    publishClouds();
  }
}

void SegmentationClient::publishClouds(){
  table_cloud_pub_.publish(table_cloud_ros_);
  object_cloud_pub_.publish(object_cloud_ros_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_lccp_client");
  ros::NodeHandle nh;
  std::string segmentation_service, cloud_topic;
  nh.param("segmentation_client/segmentation_service",segmentation_service, std::string("segmentation_service"));
  nh.param("segmentation_client/cloud_topic", cloud_topic, std::string("/camera/depth_registered/points"));
  SegmentationClient client(&nh, cloud_topic, segmentation_service);
  ros::spin();
  return 0;
}
