#include<sq_grasping/sq_grasping.h>
#include<sq_fitting/get_sq.h>
#include<sq_fitting/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<sq_fitting/sampling.h>



SQGrasping::SQGrasping(ros::NodeHandle &nh, const std::string &sq_topic, bool show_sq)
  :nh_(nh), show_sq_(show_sq)
{
  //calling sq_ server and creating sq_grasp server
  client_ = nh.serviceClient<sq_fitting::get_sq>(sq_topic);
  service_ = nh_.advertiseService("grasps", &SQGrasping::serviceCallback, this);
  std::cout<<"Started finding graps in superquadrics"<<std::endl;
  superquadrics_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sq",10);
}

SQGrasping::~SQGrasping()
{

}

void SQGrasping::sampleSQFromSQS(const sq_fitting::sqArray &sqs)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sq_cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
  sq_cloud_pcl->points.resize(0);
  for(int i=0;i<sqs.sqs.size();++i)
  {
    Sampling* sam = new Sampling(sqs.sqs[i]);
    sam->sample_pilu_fisher();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sq_ind_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
    sam->getCloud(sq_ind_pcl);
    for(size_t j=0;j<sq_ind_pcl->points.size();++j)
    {
      pcl::PointXYZRGB tmp;
      tmp.x = sq_ind_pcl->points.at(j).x;
      tmp.y = sq_ind_pcl->points.at(j).y;
      tmp.z = sq_ind_pcl->points.at(j).z;
      tmp.r =  sq_ind_pcl->points.at(j).r;
      tmp.g = sq_ind_pcl->points.at(j).g;
      tmp.b = sq_ind_pcl->points.at(j).b;
      sq_cloud_pcl->points.push_back(tmp);
    }
    delete sam;
  }
  sq_cloud_pcl->height = 1;
  sq_cloud_pcl->width = sq_cloud_pcl->points.size();
  sq_cloud_pcl->is_dense = true;
  pcl::toROSMsg(*sq_cloud_pcl, sq_cloud_);
  sq_cloud_.header.seq = 1;
  sq_cloud_.header.frame_id = sqs.header.frame_id;
  sq_cloud_.header.stamp = ros::Time::now();

}

void SQGrasping::runNode()
{
  ros::Rate rate(1);
  std::cout<<"Waiting for grasps to be called....\n";
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    superquadrics_pub_.publish(sq_cloud_);
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
  if (show_sq_)
    sampleSQFromSQS(srv.response.sqs);

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


