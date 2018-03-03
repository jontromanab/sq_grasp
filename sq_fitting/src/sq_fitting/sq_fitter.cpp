#include<sq_fitting/sq_fitter.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <pcl/filters/median_filter.h>
#include <sq_fitting/get_sq_param.h>
#include <visualization_msgs/Marker.h>
#include <sq_fitting/segment_object.h>
#include <chrono>



SQFitter::SQFitter(ros::NodeHandle &node, const std::string &segmentation_service,
                   const std::string &cloud_topic, const std::string &output_frame,
                   const SQFitter::Parameters &params)
  : nh_(node), cloud_(new PointCloud),
    filtered_cloud_(new PointCloud), cut_cloud_(new PointCloud), output_frame_(output_frame)
{

  cloud_sub_ = nh_.subscribe(cloud_topic, 1, &SQFitter::cloud_callback, this);
  table_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("table",10);
  objects_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_objects",10);
  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud",10);
  superquadrics_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("superquadrics",10);
  poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("sq_poses",10);


  cut_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cut_cloud", 10);


  client_= nh_.serviceClient<sq_fitting::segment_object>("/segmentation_service");
  service_ = nh_.advertiseService("sqs", &SQFitter::serviceCallback, this);

  this->sq_param_ = params;
  this->initialized = true;
  Objects_.resize(0);
  pVector_.resize(0);
}

SQFitter::~SQFitter(){

}

bool SQFitter::serviceCallback(sq_fitting::get_sq::Request &req, sq_fitting::get_sq::Response &res)
{
  std::cout<<"Service callback started"<<std::endl;
  std::cout<<"Request received to send back SQ"<<std::endl;
  res.sqs = sqArr_;
  res.table_center = table_center_;
  std::cout<<"Sending back: "<<sqArr_.sqs.size()<<" superquadrics";
  std::cout<<"Table center: "<<table_center_.x <<" "<<table_center_.y<<" "
          <<table_center_.z<<std::endl;
}

void SQFitter::cloud_callback(const sensor_msgs::PointCloud2& input)
{
  ROS_INFO("Cloud Received");
  pcl::fromROSMsg(input, *cloud_);
  this->input_msg_ = input;

  filterWorkSpace(this->cloud_, this->filtered_cloud_);
  pcl::toROSMsg(*filtered_cloud_, this->ws_filtered_cloud_ros_);

  getSegmentedObjects(this->filtered_cloud_);

  //auto start = std::chrono::high_resolution_clock::now();
  fitAndSample(this->Objects_,this->pVector_);
  //auto finish_fit = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed_fit = finish_fit - start;
  //std::cout<<"Elapsed toral time: "<<elapsed_fit.count()<<std::endl;


}

void SQFitter::transformFrameCloud(const CloudPtr& cloud_in, CloudPtr& cloud_out)
{
  if(output_frame_ != this->input_msg_.header.frame_id){
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
      listener.waitForTransform(output_frame_,this->input_msg_.header.frame_id,ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(output_frame_, this->input_msg_.header.frame_id,ros::Time(0), transform);

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
      pcl::transformPointCloud (*cloud_in, *cloud_out, transform_in_eigen);
      cloud_out->header.frame_id = output_frame_;
      }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      }
    }
  else
    cloud_out = cloud_in;
}

void SQFitter::transformFrameCloudBack(const CloudPtr& cloud_in, CloudPtr& cloud_out)
{
  if(output_frame_ == this->input_msg_.header.frame_id)
  {
    std::cout<<"Ok till now"<<std::endl;
  }
  else{

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{

    listener.waitForTransform( this->input_msg_.header.frame_id, output_frame_,ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(this->input_msg_.header.frame_id, output_frame_,ros::Time(0), transform);

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
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform_in_eigen);
    cloud_out->header.frame_id = this->input_msg_.header.frame_id;

  }
  catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

  }
}

void SQFitter::filterWorkSpace(const CloudPtr& cloud, CloudPtr& filtered_cloud)
{
  CloudPtr transform_cloud(new PointCloud);
  transformFrameCloud(cloud, transform_cloud);
  CloudPtr cloud_nan(new PointCloud);
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(transform_cloud);
  Eigen::Vector4f min;
  min<<sq_param_.ws_limits[0] ,sq_param_.ws_limits[2],sq_param_.ws_limits[4], 1;
  Eigen::Vector4f max;
  max<<sq_param_.ws_limits[1],sq_param_.ws_limits[3], sq_param_.ws_limits[5], 1;
  crop.setMin(min);
  crop.setMax(max);
  if(this->sq_param_.remove_nan){
    crop.filter(*cloud_nan);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_nan, *filtered_cloud, indices);
  }
  else
    crop.filter(*filtered_cloud);
}

void SQFitter::filter_StatOutlier(CloudPtr &cloud_in, CloudPtr &cloud_out)
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (cloud_in);
  sor.setMeanK (1);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_out);
}

void SQFitter::filter_RadiusOutlier(CloudPtr &cloud_in, CloudPtr &cloud_out)
{
  pcl::RadiusOutlierRemoval<PointT> outrem;
  outrem.setInputCloud(cloud_in);
  outrem.setRadiusSearch(0.05);
  outrem.setMinNeighborsInRadius (1);
  outrem.filter (*cloud_out);
}

void SQFitter::mirror_cloud(CloudPtr &cloud_in, CloudPtr &cloud_out)
{
  double x, y,z;
  getCenter(cloud_in, x, y, z);
  geometry_msgs::Pose pose_in;
  pose_in.position.x = x;
  pose_in.position.y = y;
  pose_in.position.z = z;
  pose_in.orientation.w = 1.0;

  Eigen::Affine3d pose_in_eigen;
  tf::poseMsgToEigen(pose_in, pose_in_eigen);
  Eigen::Affine3d pose_inv = pose_in_eigen.inverse();
  PointCloud::Ptr to_center(new PointCloud);
  PointCloud::Ptr cloud_mirror(new PointCloud);
  PointCloud::Ptr cloud_mirror_trns(new PointCloud);
  pcl::transformPointCloud (*cloud_in, *to_center, pose_inv);

  for(int i=0;i<to_center->points.size();++i)
  {
    PointT point;
    point.x = -(to_center->points.at(i).x);
    point.y = -(to_center->points.at(i).y);
    point.z = -(to_center->points.at(i).z);
    cloud_mirror->points.push_back(point);
  }
  cloud_mirror->header.frame_id = cloud_in->header.frame_id;

  cloud_mirror->width =  cloud_mirror->points.size();
  cloud_mirror->height = 1;
  cloud_mirror->is_dense = true;
  pcl::transformPointCloud (*cloud_mirror, *cloud_mirror_trns, pose_in_eigen);

  for(size_t i=0;i<cloud_mirror_trns->points.size();++i)
  {
    cloud_out->points.push_back(cloud_mirror_trns->points.at(i));
    cloud_out->points.push_back(cloud_in->points.at(i));
  }

  cloud_out->width =  cloud_out->points.size();
  cloud_out->height = 1;
  cloud_out->is_dense = true;
  cloud_out->header.frame_id = cloud_in->header.frame_id;
  geometry_msgs::Pose pose_out;
  transformFrame(pose_in, pose_out);
  //createCenterMarker(pose_out.position.x, pose_out.position.y, pose_out.position.z );
}

void SQFitter::transformFrame(const geometry_msgs::Pose &pose_in, geometry_msgs::Pose& pose_out)
{

  if(output_frame_ == this->input_msg_.header.frame_id)
  {
    std::cout<<"Ok till now"<<std::endl;
    pose_out = pose_in;
  }
  else{

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{

    listener.waitForTransform( output_frame_,this->input_msg_.header.frame_id,ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(output_frame_, this->input_msg_.header.frame_id,ros::Time(0), transform);


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
  catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

  }
}

void SQFitter::getSegmentedObjects(CloudPtr& cloud)
{
  Objects_.resize(0);
  std::vector<CloudPtr> segmented_clouds;
  CloudPtr transform_cloud(new PointCloud);
  transformFrameCloudBack(cloud, transform_cloud);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*transform_cloud, cloud_msg);
  sq_fitting::segment_object srv;
  srv.request.input_cloud = cloud_msg;
  if(client_.call(srv)){
    table_cloud_ = srv.response.plane_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> segmented_objects_cloud;
    for(int i=0;i<srv.response.object_cloud.size();++i){
      CloudPtr tmp(new PointCloud);
      pcl::fromROSMsg(srv.response.object_cloud[i], *tmp);
      segmented_clouds.push_back(tmp);
      float r = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      float g = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      float b = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      for(int j=0;j<tmp->points.size();++j){
        pcl::PointXYZRGB temp_point;
        temp_point.x = tmp->points.at(j).x;
        temp_point.y = tmp->points.at(j).y;
        temp_point.z = tmp->points.at(j).z;
        temp_point.r = r * 255;
        temp_point.g = g * 255;
        temp_point.b = b * 255;
        segmented_objects_cloud.points.push_back(temp_point);
      }
    }
    segmented_objects_cloud.width = segmented_objects_cloud.points.size();
    segmented_objects_cloud.height = 1;
    segmented_objects_cloud.is_dense = true;
    CloudPtr transform_cloud_back(new PointCloud);
    transformFrameCloud(segmented_objects_cloud.makeShared(), transform_cloud_back);
    pcl::toROSMsg(*transform_cloud_back, objects_cloud_ros_);
    objects_cloud_ros_.header.seq = 1;
    objects_cloud_ros_.header.frame_id = this->output_frame_;
    objects_cloud_ros_.header.stamp = ros::Time::now();
  }

  //transforming every segmented point cloud to output frame
  for(int i=0;i<segmented_clouds.size();++i){
    CloudPtr temp_cloud(new PointCloud);
    transformFrameCloud(segmented_clouds[i], temp_cloud);
    Objects_.push_back(temp_cloud);
  }
}

void SQFitter::fitAndSampleTh(CloudPtr &cloud_in,std::string& method, ParamMultiVector& pvector){
  std::unique_ptr<SuperquadricFitting> fit(new SuperquadricFitting(cloud_in));
  if(!fit->set_pose_est_method(method))
    ROS_ERROR("Method not recognized");
  fit->fit();
  sq_fitting::sq min_param;
  fit->getMinParams(min_param);

  std::unique_ptr<Sampling> samp(new Sampling(min_param));
  CloudPtr sq_cloud(new PointCloud);
  samp->sample_pilu_fisher();
  samp->getCloud(sq_cloud);
  std::lock_guard<std::mutex> blck(mu_);
  pvector.push_back(std::make_pair(min_param, sq_cloud));
}

void SQFitter::fitAndSample(std::vector<CloudPtr>& objs, ParamMultiVector& pvector){
  pvector.clear();
  pvector.reserve(objs.size());
  poseArr_.poses.resize(0);
  sqArr_.sqs.resize(0);
  std::vector<std::thread> threads;
  for(int i=0;i<this->Objects_.size();++i)
  {
    threads.push_back(std::thread(&SQFitter::fitAndSampleTh, this, std::ref(objs[i]),
                      std::ref(sq_param_.pose_est_method),std::ref(pvector)));
  }
  for(auto &t:threads)
    t.join();

  CloudPtr sq_cloud_pcl(new PointCloud);
  for(ParamMultiVector::iterator it = pvector.begin(); it !=pvector.end();++it){
    *sq_cloud_pcl+=*(it->second);
    sq_fitting::sq param = it->first;
    poseArr_.poses.push_back(param.pose);
    sqArr_.sqs.push_back(param);
  }
  ROS_INFO("Fitted %lu Objects",pvector.size());
  //std::cout<<"Size of cloud: "<<sq_cloud_pcl->points.size()<<std::endl;
  sq_cloud_pcl->height = 1;
  sq_cloud_pcl->width = sq_cloud_pcl->points.size();
  sq_cloud_pcl->is_dense = true;
  pcl::toROSMsg(*sq_cloud_pcl, sq_cloud_);
  sq_cloud_.header.seq = 1;
  sq_cloud_.header.frame_id = output_frame_;
  sq_cloud_.header.stamp = ros::Time::now();

  sqArr_.header.frame_id = output_frame_;
  sqArr_.header.stamp = ros::Time::now();
  poseArr_.header.frame_id =  output_frame_;
  poseArr_.header.stamp = ros::Time::now();

}

void SQFitter::publishClouds()
{
  table_pub_.publish(table_cloud_);
  objects_pub_.publish(objects_cloud_ros_);
  filtered_cloud_pub_.publish(ws_filtered_cloud_ros_);
  superquadrics_pub_.publish(sq_cloud_);
  poses_pub_.publish(poseArr_);


  cut_cloud_pub_.publish(cut_cloud_ros_);
}


void SQFitter::fit()
{
  if(this->initialized = true)
  {
    ros::Rate rate(1);
    //ROS_INFO("Fitting superquadrics");
    while(ros::ok())
    {
      publishClouds();
      ros::spinOnce();
      rate.sleep();
    }
  }
}




















