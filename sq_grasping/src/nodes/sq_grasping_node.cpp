#include <ros/ros.h>
#include<sq_grasping/sq_grasping.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "super");
  ros::NodeHandle nh_("~");

  std::string sq_topic;
  nh_.getParam("sq_topic", sq_topic);

  SQGrasping sqgrasping(nh_, sq_topic);
  sqgrasping.runNode();
  return 0;



}
