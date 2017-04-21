#include<sq_grasping/sq_grasping.h>

SQGrasping::SQGrasping(ros::NodeHandle &nh, const std::string &sq_topic)
  :nh_(nh)
{

  //Subscriber for superquadrics
  sq_sub_ = nh_.subscribe(sq_topic, 10, &SQGrasping::sqCallback, this);
  //ROS service for grasps
  service_ = nh_.advertiseService("sq_grasps", &SQGrasping::serviceCallback, this);
  sqs_.sqs.resize(0);
  new_sqs_.sqs.resize(0);
  std::cout<<"Started finding graps in superquadrics"<<std::endl;
}

SQGrasping::~SQGrasping()
{

}


void SQGrasping::runNode()
{
  ros::Rate rate(1);
  std::cout<<"Waiting for sq topic....\n";
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

void SQGrasping::sqCallback(const sq_fitting::sqArray& msg)
{
  new_sqs_ = msg;
}


bool SQGrasping::serviceCallback(sq_grasping::getGrasps::Request &req, sq_grasping::getGrasps::Response &res)
{

  std::cout<<"Service callback started"<<std::endl;
  sqs_ = new_sqs_;
  std::cout<<"Got "<<sqs_.sqs.size()<<" superquadrics\n";
  if(sqs_.sqs.size() ==0)
  {
    std::cout<<"No more superquadrics available to grasp \n";
    std::cout<<"Waiting for new superquadrics\n";
    return false;
  }

  //creating grasps for superquadrics
  std::cout<<"Dealing with a hand with :"<<req.num_of_fingers<<" fingers\n";
  createGrasps(sqs_, res.grasps);
  std::cout<<"Created response with "<<(int)res.grasps.grasps.size()<<" grasps\n";

}


