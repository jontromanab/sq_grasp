#include <ros/ros.h>
#include<sq_grasping/sq_grasping.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "super");
  ros::NodeHandle nh_("~");

  std::string sq_topic;
  bool show_sq, show_grasp;
  std::string output_frame;
  std::string ee_group;
  std::string ee_grasp_link;
  std::string arm_group;

  nh_.getParam("sq_topic", sq_topic);
  nh_.getParam("show_sq", show_sq);
  nh_.getParam("show_grasp", show_grasp);
  nh_.getParam("output_frame", output_frame);

  nh_.getParam("ee_group", ee_group);
  nh_.getParam("ee_grasp_link", ee_grasp_link);

  nh_.getParam("arm_group", arm_group);

  SQGrasping sqgrasping(nh_, sq_topic, show_sq, show_grasp,
                        output_frame, ee_group, ee_grasp_link, arm_group);

  sqgrasping.runNode();
  return 0;



}
