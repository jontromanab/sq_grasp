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
#include<visualization_msgs/MarkerArray.h>

#include<grasp_execution/create_gripper_marker.h>



SQGrasping::SQGrasping(ros::NodeHandle &nh, const std::string &sq_topic, bool show_sq, bool show_grasp,
                       const std::string output_frame, const std::string ee_group,const std::string ee_grasp_link,
                       const std::string arm_group)
  :nh_(nh), spinner(1) ,show_sq_(show_sq), show_grasp_(show_grasp),
    output_frame_(output_frame), ee_group_(ee_group),
    ee_grasp_link_(ee_grasp_link), arm_group_(arm_group)
{
  //calling sq_ server and creating sq_grasp server
  client_ = nh.serviceClient<sq_fitting::get_sq>(sq_topic);
  service_ = nh_.advertiseService("grasps", &SQGrasping::serviceCallback, this);
  superquadrics_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sq",10);
  grasp_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("grasps",10);
  grasp_pub_arrow_ = nh_.advertise<visualization_msgs::MarkerArray>("grasps_arrow",10);

  ee_name_ = ee_group;
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
    grasp_pub_arrow_.publish(poses_arrow_);
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

void SQGrasping::createGraspsMarkers(const grasp_execution::graspArr &grasps, visualization_msgs::MarkerArray& markers)
{
  markers.markers.resize(0);
  for (int i=0;i<grasps.grasps.size();++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = grasps.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "arrow";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = grasps.grasps[i].pose;
    marker.scale.x = 0.05;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    markers.markers.push_back(marker);
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
  CreateGrasps* create = new CreateGrasps(new_sqArr, arm_group_,ee_grasp_link_);
  create->sample_initial_grasps();
  grasp_execution::graspArr grasps;
  create->getGrasps(grasps);
  delete create;

  //Creating poseArray for visualization.
  std::cout<<"We have: "<<grasps.grasps.size()<<" grasps."<<std::endl;
  if(show_grasp_)
  {
    createGraspsMarkers(grasps, poses_arrow_);
    for (int i=0;i<grasps.grasps.size();++i)
    {
      visualization_msgs::MarkerArray markArr = createGripperMarkerMoveIT(grasps.grasps[i].pose, 0.0, i+1);
      for(int j=0;j<markArr.markers.size();++j)
      {
        poses_.markers.push_back(markArr.markers[j]);
      }
    }
  }
  //
  //Showing grasps as gripper instead of arrow for better understanding.


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

visualization_msgs::MarkerArray SQGrasping::createGripperMarkerMoveIT(geometry_msgs::Pose pose, double opening_angle, int id)
{
  std::string ee_name = ee_group_.getName();
  moveit::core::RobotModelConstPtr robot_model = ee_group_.getRobotModel();
  moveit::core::RobotStatePtr robot_state_(new moveit::core::RobotState(robot_model));

  //Setting the gripper joint to the openning value of the gripper
  const std::string ee_joint = "l_gripper_l_finger_joint";
  double value = opening_angle;
  double *valuePtr = &value;
  robot_state_->setJointPositions(ee_joint, valuePtr);
  robot_state_->update();

  //New jointmodel group of the end effector with the opening value
  const moveit::core::JointModelGroup* ee_jmp = robot_model->getJointModelGroup(ee_name);
  if(ee_jmp == NULL)
  {
    ROS_ERROR_STREAM("Unable to find joint model group with address"<<ee_jmp);
    //return false;
  }

  //maps
  std::map<const robot_model::JointModelGroup *, visualization_msgs::MarkerArray> ee_markers_map_;
  ee_markers_map_[ee_jmp].markers.clear();
  const std::vector<std::string>& ee_link_names = ee_jmp->getLinkModelNames();
  robot_state_->getRobotMarkers(ee_markers_map_[ee_jmp], ee_link_names);
  const std::string& ee_parent_link_name = ee_jmp->getEndEffectorParentGroup().second;

  Eigen::Affine3d tf_root_to_ee = robot_state_->getGlobalLinkTransform(ee_parent_link_name);
  Eigen::Affine3d tf_ee_to_root = tf_root_to_ee.inverse();
  Eigen::Affine3d trans_bw_poses;
  trans_bw_poses = tf_root_to_ee * tf_ee_to_root;

  Eigen::Affine3d grasp_tf;
  tf::poseMsgToEigen(pose, grasp_tf);
  for(std::size_t i=0; i<ee_markers_map_[ee_jmp].markers.size();++i)
  {
    ee_markers_map_[ee_jmp].markers[i].header.frame_id = output_frame_;
    ee_markers_map_[ee_jmp].markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    ee_markers_map_[ee_jmp].markers[i].mesh_use_embedded_materials = true;
    ee_markers_map_[ee_jmp].markers[i].id = ((i+5)*100*id)-id;

    ee_markers_map_[ee_jmp].markers[i].header.stamp = ros::Time::now();
    ee_markers_map_[ee_jmp].markers[i].ns = "gripper_links";
    ee_markers_map_[ee_jmp].markers[i].lifetime = ros::Duration(40.0);

    ee_markers_map_[ee_jmp].markers[i].color.r = 0.5;
    ee_markers_map_[ee_jmp].markers[i].color.g = 0.5;
    ee_markers_map_[ee_jmp].markers[i].color.b = 0.0;
    ee_markers_map_[ee_jmp].markers[i].color.a = 0.5;

    Eigen::Affine3d link_marker;
    tf::poseMsgToEigen(ee_markers_map_[ee_jmp].markers[i].pose, link_marker);

    Eigen::Affine3d tf_link_in_root =  tf_ee_to_root * link_marker;

    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg( grasp_tf * tf_link_in_root  , new_marker_pose );
    ee_markers_map_[ee_jmp].markers[i].pose = new_marker_pose;
  }
  return ee_markers_map_[ee_jmp];
}
