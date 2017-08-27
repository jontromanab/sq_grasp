#include <ros/ros.h>
#include<sq_grasping/sq_grasping.h>

static const bool SHOW_SQ = true;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "super");
  ros::NodeHandle nh_("~");

  std::string sq_topic;
  bool show_sq;
  std::string output_frame;
  nh_.getParam("sq_topic", sq_topic);
  nh_.getParam("show_sq", show_sq);
  nh_.getParam("output_frame", output_frame);

  SQGrasping sqgrasping(nh_, sq_topic, show_sq, output_frame);
  sqgrasping.runNode();
  return 0;



}
