#include <ros/ros.h>
#include <sq_grasping/getGrasps.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <grasp_execution/ExecuteGraspAction.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_grasps_node");
  ros::NodeHandle nh;

  //Creating a service client to get grasps from sq_grasp service
  ros::ServiceClient client = nh.serviceClient<sq_grasping::getGrasps>("/grasping/sq_grasps");
  sq_grasping::getGrasps srv;
  srv.request.num_of_fingers = 1;




  if(client.call(srv))
  {
    std::cout<<"Got "<<srv.response.grasps.grasps.size()<<" grasps\n";
  }
  else
  {
    ROS_ERROR("Failed to call service getGrasps\n");
    return 1;
  }

  /*std::cout<<"Printing grasps\n";
  for(int i=0;i<srv.response.grasps.grasps.size();++i)
  {
    std::cout<<"Angle: "<<srv.response.grasps.grasps[i].angle<<std::endl;
    std::cout<<"Pose:  "<<srv.response.grasps.grasps[i].pose.position.x<<
            " "<<srv.response.grasps.grasps[i].pose.position.y<<" "<<srv.response.grasps.grasps[i].pose.position.z<<std::endl;
  }*/


  //Creating action server client to send grasps to action server
  actionlib::SimpleActionClient<grasp_execution::ExecuteGraspAction> gs("grasp_execution_server", true);
  ROS_INFO("Waiting for server to start");
  gs.waitForServer ();
  ROS_INFO("Grasp Execution Action server started, sending grasp");
  grasp_execution::ExecuteGraspGoal goal;
  goal.grasp = srv.response.grasps.grasps[0];
  gs.sendGoal(goal);

  bool finished_before_timeout = gs.waitForResult (ros::Duration(40.0));
    if(finished_before_timeout )
    {
      actionlib::SimpleClientGoalState state = gs.getState ();
      ROS_INFO("Action finished: %s", state.toString ().c_str ());
    }

    else
      ROS_INFO("Action did not finish before time out");


  return 0;

}
