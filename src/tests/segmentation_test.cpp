#include <ros/ros.h>
#include <sq_grasp/segmentation.h>


const std::string CLOUD_TOPIC = "/camera/depth_registered/points";
const double ZMAX = 2.0;
const double ZMIN = 0.02;

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "segmentation_test");
  ros::NodeHandle nh("~");

  Segmentation::Parameters params;
  std::string cloud_topic;
  double zmin;
  double zmax;
  nh.param("cloud_topic", cloud_topic, CLOUD_TOPIC);
  nh.param("zmin", params.zmin_, ZMIN);
  nh.param("zmax", params.zmax_, ZMAX);

  Segmentation seg(nh, cloud_topic, params);
  seg.segment();

  ros::spin();

}
