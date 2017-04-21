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
  client.call(srv);
  std::cout<<"Got "<<srv.response.grasps.grasps.size()<<" grasps\n";


  actionlib::SimpleActionClient<grasp_execution::ExecuteGraspAction> gs("grasp_execution_server", true);
  ROS_INFO("Waiting for server to start");
  gs.waitForServer ();

  for(int i=0;i<srv.response.grasps.grasps.size();++i)
  {
    sq_grasping::getGrasps new_srv;
    new_srv.request.num_of_fingers = 1;
    client.call(new_srv);
    grasp_execution::ExecuteGraspGoal goal;
    goal.grasp = new_srv.response.grasps.grasps[0];
    gs.sendGoal(goal);
    bool finished_before_timeout = gs.waitForResult (ros::Duration(40.0));
    if(finished_before_timeout )
    {
      ROS_INFO("Action finished");
    }
    else
      ROS_INFO("Action did not finish before time out");
  }
  /*while(srv.response.grasps.grasps.size()>0)
  {
    sq_grasping::getGrasps new_srv;
    new_srv.request.num_of_fingers = 1;
    client.call(new_srv);
    grasp_execution::ExecuteGraspGoal goal;
    goal.grasp = new_srv.response.grasps.grasps[0];
    gs.sendGoal(goal);
    bool finished_before_timeout = gs.waitForResult (ros::Duration(40.0));
    if(finished_before_timeout )
    {
      ROS_INFO("Action finished");
    }
    else
      ROS_INFO("Action did not finish before time out");
  }*/






  /*if(client.call(srv))
  {
    std::cout<<"Got "<<srv.response.grasps.grasps.size()<<" grasps\n";
  }
  else
  {
    ROS_ERROR("Failed to call service getGrasps\n");
    return 1;
  }

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
      ROS_INFO("Action did not finish before time out");*/


  return 0;

}
