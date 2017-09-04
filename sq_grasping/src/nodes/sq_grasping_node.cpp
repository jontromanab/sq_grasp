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
  std::string ee_joint;
  double ee_max_opening_angle;
  std::string arm_group;
  double approach_value;
  double object_padding;


  nh_.getParam("sq_topic", sq_topic);
  nh_.getParam("show_sq", show_sq);
  nh_.getParam("show_grasp", show_grasp);
  nh_.getParam("output_frame", output_frame);

  nh_.getParam("ee_group", ee_group);
  nh_.getParam("ee_grasp_link", ee_grasp_link);
  nh_.getParam("ee_joint",ee_joint);
  nh_.getParam("ee_max_opening_angle",ee_max_opening_angle);

  nh_.getParam("arm_group", arm_group);
  nh_.getParam("approach_value", approach_value);
  nh_.getParam("object_padding", object_padding);

  SQGrasping sqgrasping(nh_, sq_topic, show_sq, show_grasp,
                        output_frame, ee_group, ee_grasp_link, ee_joint, ee_max_opening_angle,
                        object_padding, arm_group,approach_value);

  sqgrasping.runNode();
  return 0;



}
