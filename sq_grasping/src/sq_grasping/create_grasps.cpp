#include<sq_grasping/create_grasps.h>

CreateGrasps::CreateGrasps(sq_fitting::sqArray sqArr)
{
  sqArr_ = sqArr;\
  init_grasps_.grasps.resize(0);
  frame_id_ = sqArr_.header.frame_id;
}

CreateGrasps::~CreateGrasps()
{

}

void CreateGrasps::sample_initial_grasps()
{
  init_grasps_.header.frame_id = frame_id_;
  init_grasps_.header.stamp = ros::Time::now();
  for(int i=0;i<sqArr_ .sqs.size();++i)
    {
      grasp_execution::grasp gr;
      gr.pose = sqArr_ .sqs[i].pose;
      gr.pose.position.z = sqArr_.sqs[i].pose.position.z + 0.14;
      gr.pose.orientation.x = 0.639;
      gr.pose.orientation.y = -0.326;
      gr.pose.orientation.z = -0.616;
      gr.pose.orientation.w = -0.327;
      gr.angle = 0.79;
      gr.approach.x = 0.0;
      gr.approach.y = 0.0;
      gr.approach.z = -1.0;
      init_grasps_.grasps.push_back(gr);
    }


}

void CreateGrasps::getGrasps(grasp_execution::graspArr &grasps)
{
  grasps = init_grasps_;
}
