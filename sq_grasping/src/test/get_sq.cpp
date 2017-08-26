#include <ros/ros.h>
#include <sq_grasping/getGrasps.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_grasps_node");
  ros::NodeHandle nh;

  //Creating a service client to get grasps from sq_grasp service
  ros::ServiceClient client = nh.serviceClient<sq_grasping::getGrasps>("/grasping/sq_grasps");
  sq_grasping::getGrasps srv;

  srv.request.num_of_fingers = 1;
  client.call(srv);
  std::cout<<"Got "<<srv.response.grasps.grasps.size()<<" grasps\n";

  return 0;


}
