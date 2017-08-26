#include <ros/ros.h>
#include<sq_grasping/sq_grasping.h>

static const bool SHOW_SQ = true;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "super");
  ros::NodeHandle nh_("~");

  std::string sq_topic;
  bool show_sq;
  nh_.getParam("sq_topic", sq_topic);
  nh_.getParam("show_sq", show_sq);

  SQGrasping sqgrasping(nh_, sq_topic, show_sq);
  sqgrasping.runNode();
  return 0;



}
