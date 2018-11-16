#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sq_test_node");
  ros::NodeHandle nh;

  std::cout<< "c++ from ros node" <<std::endl;

  ROS_INFO("test the git");
}
