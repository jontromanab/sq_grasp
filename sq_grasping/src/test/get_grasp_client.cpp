#include <ros/ros.h>
#include<sq_grasping/getGrasps.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_grasp_test");
  ros::NodeHandle nh;

  //Creating a service client to get grasps from sq_grasp service
  ros::ServiceClient client = nh.serviceClient<sq_grasping::getGrasps>("/super/sq_grasps");
  sq_fitting::get_sq srv;


  srv.request.running = true;
  client.call(srv);
  std::cout<<"Got "<<srv.response.sqs.sqs.size()<<" superquadrics"<<std::endl;

  return 0;


}
