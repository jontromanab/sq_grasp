#include<sq_grasping/sq_grasping.h>
#include<sq_fitting/get_sq.h>
#include<sq_fitting/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<sq_fitting/sampling.h>
#include<tf/transform_listener.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include<sq_grasping/create_grasps.h>



SQGrasping::SQGrasping(ros::NodeHandle &nh, const std::string &sq_topic, bool show_sq, const std::string output_frame)
  :nh_(nh), show_sq_(show_sq), output_frame_(output_frame)
{
  //calling sq_ server and creating sq_grasp server
  client_ = nh.serviceClient<sq_fitting::get_sq>(sq_topic);
  service_ = nh_.advertiseService("grasps", &SQGrasping::serviceCallback, this);
  superquadrics_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sq",10);
  grasp_pub_ = nh_.advertise<geometry_msgs::PoseArray>("grasps",10);
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
    grasp_pub_.publish(poses_);
  }
}

bool SQGrasping::checkForTransform(const std::string& input_frame, const std::string& output_frame)
{
  if (input_frame == output_frame)
      return true;
  else
    return false;
}

void sq_create_transform(const geometry_msgs::Pose& pose, Eigen::Affine3f& transform)
{
  transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.translation()<<pose.position.x, pose.position.y, pose.position.z;
  transform.rotate(q);
}

void SQGrasping::transformFrame(const std::string& input_frame, const std::string& output_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
    try
    {
      listener.waitForTransform(input_frame, output_frame, ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(input_frame, output_frame, ros::Time(0), transform);
      geometry_msgs::Pose inter_pose;
      inter_pose.position.x = transform.getOrigin().x();
      inter_pose.position.y = transform.getOrigin().y();
      inter_pose.position.z = transform.getOrigin().z();
      inter_pose.orientation.x = transform.getRotation().getX();
      inter_pose.orientation.y = transform.getRotation().getY();
      inter_pose.orientation.z = transform.getRotation().getZ();
      inter_pose.orientation.w = transform.getRotation().getW();
      Eigen::Affine3d transform_in_eigen;
      tf::poseMsgToEigen(inter_pose, transform_in_eigen);
      Eigen::Affine3f pose_in_eigen;
      sq_create_transform(pose_in, pose_in_eigen);
      tf::poseEigenToMsg( transform_in_eigen * pose_in_eigen.cast<double>(), pose_out );
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

void SQGrasping::createGrasps(const sq_fitting::sqArray& sqs , grasp_execution::graspArr& gs)
{
  sq_fitting::sqArray new_sqArr;
  new_sqArr.header.frame_id = output_frame_;
  new_sqArr.header.stamp = ros::Time::now();
  for (int i=0;i<sqs.sqs.size();++i)
    {
      sq_fitting::sq new_sq;
      geometry_msgs::Pose output_pose;
      if(!checkForTransform(sqs.header.frame_id, output_frame_))
      {
        transformFrame(sqs.header.frame_id, output_frame_, sqs.sqs[i].pose, output_pose);
        new_sq.pose = output_pose;
      }
      else
        new_sq.pose = sqs.sqs[i].pose;
      new_sq.a1 = sqs.sqs[i].a1;
      new_sq.a2 = sqs.sqs[i].a2;
      new_sq.a3 = sqs.sqs[i].a3;
      new_sq.e1 = sqs.sqs[i].e1;
      new_sq.e2 = sqs.sqs[i].e2;
      new_sqArr.sqs.push_back(new_sq);
  }
  sampleSQFromSQS(new_sqArr);
  CreateGrasps* create = new CreateGrasps(new_sqArr);
  create->sample_initial_grasps();
  grasp_execution::graspArr grasps;
  create->getGrasps(grasps);
  delete create;

  //Creating poseArray for visualization.
  poses_.header.frame_id = grasps.header.frame_id;
  poses_.header.stamp = ros::Time::now();
  for (int i=0;i<grasps.grasps.size();++i)
  {
    poses_.poses.push_back(grasps.grasps[i].pose);
  }

  gs = grasps;

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
  /*if (show_sq_)
    //sampleSQFromSQS(srv.response.sqs);
    sampleSQFromSQS(new_sqs_);*/

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


