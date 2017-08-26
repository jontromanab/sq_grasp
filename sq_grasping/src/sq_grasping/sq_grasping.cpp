#include<sq_grasping/sq_grasping.h>
#include<sq_fitting/get_sq.h>

SQGrasping::SQGrasping(ros::NodeHandle &nh, const std::string &sq_topic)
  :nh_(nh)
{
  //calling sq_ server and creating sq_grasp server
  client_ = nh.serviceClient<sq_fitting::get_sq>(sq_topic);
  service_ = nh_.advertiseService("sq_grasps", &SQGrasping::serviceCallback, this);
  std::cout<<"Started finding graps in superquadrics"<<std::endl;
}

SQGrasping::~SQGrasping()
{

}


void SQGrasping::runNode()
{
  ros::Rate rate(1);
  std::cout<<"Waiting for grasps to be called....\n";
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void createGrasps(const sq_fitting::sqArray& sqs , grasp_execution::graspArr& gs)
{
  for(int i=0;i<sqs.sqs.size();++i)
  {
    grasp_execution::grasp gr;
    gr.pose = sqs.sqs[i].pose;
    gr.pose.position.z = sqs.sqs[i].pose.position.z + 0.14;
    gr.pose.orientation.x = 0.639;
    gr.pose.orientation.y = -0.326;
    gr.pose.orientation.z = -0.616;
    gr.pose.orientation.w = -0.327;
    gr.angle = 0.79;
    gr.approach.x = 0.0;
    gr.approach.y = 0.0;
    gr.approach.z = -1.0;
    gs.grasps.push_back(gr);
  }
  gs.header.frame_id = sqs.header.frame_id;
  gs.header.stamp = ros::Time::now();
}


bool SQGrasping::serviceCallback(sq_grasping::getGrasps::Request &req, sq_grasping::getGrasps::Response &res)
{
  sqs_.sqs.resize(0);
  std::cout<<"Service callback started"<<std::endl;
  std::cout<<"Dealing with a hand with :"<<req.num_of_fingers<<" fingers\n";
  std::cout<<"I am calling the sq_fitting topic for SQ"<<std::endl;
  sq_fitting::get_sq srv;
  srv.request.running = true;
  client_.call(srv);
  std::cout<<"Received: "<<srv.response.sqs.sqs.size()<<" superquadrics"<<std::endl;

  sqs_ = srv.response.sqs;
  if(sqs_.sqs.size() ==0)
  {
    std::cout<<"No more superquadrics available to grasp \n";
    std::cout<<"Waiting for new superquadrics\n";
    return false;
  }

  //creating grasps for superquadrics

  createGrasps(sqs_, res.grasps);
  std::cout<<"Created response with "<<(int)res.grasps.grasps.size()<<" grasps\n";

}


