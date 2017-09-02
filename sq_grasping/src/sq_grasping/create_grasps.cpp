#include <ros/ros.h>
#include<sq_grasping/create_grasps.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include<tf/transform_listener.h>

#include<moveit_msgs/GetPositionIK.h>
#include<moveit_msgs/PositionIKRequest.h>

CreateGrasps::CreateGrasps(ros::NodeHandle &nh, sq_fitting::sqArray sqArr, const std::string group,const std::string ee_name,
                           double ee_max_opening_angle): nh_(nh), group_(group),
  ee_name_(ee_name), ee_max_opening_angle_(ee_max_opening_angle)
{
  sqArr_ = sqArr;
  init_grasps_.grasps.resize(0);
  frame_id_ = sqArr_.header.frame_id;
  std::cout<<"frame_id: "<<sqArr_.header.frame_id<<std::endl;
  createTransform(ee_name);
  client_= nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
}

CreateGrasps::~CreateGrasps()
{

}

void CreateGrasps::createTransform(const std::string &grasp_frame)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  std::string planning_frame = group_.getEndEffectorLink();
  try
  {
    listener.waitForTransform(grasp_frame, planning_frame, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(grasp_frame, planning_frame, ros::Time(0), transform);
    geometry_msgs::Pose inter_pose;
    inter_pose.position.x = transform.getOrigin().x();
    inter_pose.position.y = transform.getOrigin().y();
    inter_pose.position.z = transform.getOrigin().z();
    inter_pose.orientation.x = transform.getRotation().getX();
    inter_pose.orientation.y = transform.getRotation().getY();
    inter_pose.orientation.z = transform.getRotation().getZ();
    inter_pose.orientation.w = transform.getRotation().getW();
    tf::poseMsgToEigen(inter_pose, transform_);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

}

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

Eigen::Affine3d create_transformation_matrix(const double tx, const double ty, const double tz, const double ax, const double ay, const double az)
{
  Eigen::Affine3d rot_matrix =create_rotation_matrix(ax, ay, az);
  Eigen::Affine3d translation_matrix(Eigen::Translation3d(Eigen::Vector3d(tx, ty, tz)));
  Eigen::Affine3d trns_mat = translation_matrix*rot_matrix;// * translation_matrix;
  return trns_mat;
}

geometry_msgs::Pose rotatePose(const geometry_msgs::Pose pose, double value, int dir, bool negative)
{
  tf2::Transform trns;
  tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Vector3 vec(pose.position.x, pose.position.y, pose.position.z);
  trns.setOrigin (vec);
  trns.setRotation (quat);

  tf2::Quaternion quat2;

  int ve_value = -1;
  if(negative)
    ve_value = 1;

  if(dir==1)
    quat2.setRPY(ve_value*value, 0,  0);
  if(dir==2)
    quat2.setRPY(0,ve_value*value, 0);
  if(dir==3)
    quat2.setRPY(0,0,ve_value*value);
  tf2::Vector3 vec2(0, 0 , 0);
  tf2::Transform multiplier;
  //multiplier.setIdentity ();
  multiplier.setOrigin (vec2);
  multiplier.setRotation (quat2);

  trns = trns * multiplier;
  tf2::Vector3 new_pose_vec;
  tf2::Quaternion new_pose_quat;
  new_pose_vec = trns.getOrigin ();
  new_pose_quat = trns.getRotation ();
  new_pose_quat.normalize ();

  geometry_msgs::Pose new_pose;
  new_pose.position.x = new_pose_vec[0];
  new_pose.position.y = new_pose_vec[1];
  new_pose.position.z = new_pose_vec[2];
  new_pose.orientation.x = new_pose_quat[0];
  new_pose.orientation.y = new_pose_quat[1];
  new_pose.orientation.z = new_pose_quat[2];
  new_pose.orientation.w = new_pose_quat[3];
  return new_pose;
}

void CreateGrasps::createInitGrasps(const sq_fitting::sq &sq, std::vector<grasp_execution::grasp>& grasps)
{
  //First transform the pose to the origin
  Eigen::Affine3d pose_in_eigen;
  tf::poseMsgToEigen(sq.pose, pose_in_eigen);
  Eigen::Affine3d pose_inv = pose_in_eigen.inverse();
  Eigen::Affine3d pose_in_center = pose_in_eigen * pose_inv;

  //First grasp in x direction
  grasp_execution::grasp gr_x_pos;
  Eigen::Affine3d translation_matrix_orig(Eigen::Translation3d(Eigen::Vector3d(-sq.a1, 0, 0)));
  Eigen::Affine3d pose_in_center_trnsl1 = translation_matrix_orig* pose_in_center;
  Eigen::Affine3d back_to_place1 = pose_in_eigen * pose_in_center_trnsl1 ;
  Eigen::Affine3d back_to_place1_trnsformed = back_to_place1*transform_;
  geometry_msgs::Pose orig_pose;
  tf::poseEigenToMsg(back_to_place1_trnsformed, orig_pose);
  gr_x_pos.pose = orig_pose;
  gr_x_pos.angle = sq.a2*10;
  grasps.push_back(gr_x_pos);

  //Second grasp in x direction
  grasp_execution::grasp gr_x_neg;
  Eigen::Affine3d transformation_mat_x =  create_transformation_matrix(sq.a1, 0, 0, 0, 0, M_PI);
  Eigen::Affine3d pose_in_center_inv_x = pose_in_center * transformation_mat_x;
  Eigen::Affine3d back_to_place2 = pose_in_eigen * pose_in_center_inv_x ;
  Eigen::Affine3d back_to_place2_trnsformed = back_to_place2*transform_;
  geometry_msgs::Pose inverted_pose_x;
  tf::poseEigenToMsg(back_to_place2_trnsformed, inverted_pose_x);
  gr_x_neg.pose = inverted_pose_x;
  gr_x_neg.angle = sq.a2*10;
  grasps.push_back(gr_x_neg);

  //First grasp in Y direction
  grasp_execution::grasp gr_y_pos;
  Eigen::Affine3d transformation_mat_y =  create_transformation_matrix(0, -sq.a2, 0, 0, 0 , M_PI/2);
  Eigen::Affine3d pose_in_center_y = pose_in_center * transformation_mat_y;
  Eigen::Affine3d back_to_place3 = pose_in_eigen * pose_in_center_y ;
  Eigen::Affine3d back_to_place3_trnsformed = back_to_place3*transform_;
  geometry_msgs::Pose pose_y;
  tf::poseEigenToMsg(back_to_place3_trnsformed, pose_y);
  gr_y_pos.pose = pose_y;
  gr_y_pos.angle = sq.a1*10;
  grasps.push_back(gr_y_pos);


  //Second Grasp in Y direction
  grasp_execution::grasp gr_y_neg;
  Eigen::Affine3d transformation_mat_inv_y =  create_transformation_matrix(0, sq.a2, 0, 0, 0 , -M_PI/2);
  Eigen::Affine3d pose_in_center_inv_y = pose_in_center * transformation_mat_inv_y;
  Eigen::Affine3d back_to_place4 = pose_in_eigen * pose_in_center_inv_y ;
  Eigen::Affine3d back_to_place4_trnsformed = back_to_place4*transform_;
  geometry_msgs::Pose pose_inv_y;
  tf::poseEigenToMsg(back_to_place4_trnsformed, pose_inv_y);
  gr_y_neg.pose = pose_inv_y;
  gr_y_neg.angle = sq.a1*10;
  grasps.push_back(gr_y_neg);


  //First grasp in Z direction
  grasp_execution::grasp gr_z_pos;
  Eigen::Affine3d transformation_mat_z =  create_transformation_matrix(0, 0,sq.a3, 0,  M_PI/2, 0);
  Eigen::Affine3d pose_in_center_z = pose_in_center * transformation_mat_z;
  Eigen::Affine3d back_to_place5 = pose_in_eigen * pose_in_center_z ;
  Eigen::Affine3d back_to_place5_trnsformed = back_to_place5*transform_;
  geometry_msgs::Pose pose_z;
  tf::poseEigenToMsg(back_to_place5_trnsformed, pose_z);
  gr_z_pos.pose = pose_z;
  gr_z_pos.angle = sq.a2*10;
  grasps.push_back(gr_z_pos);

  //Second grasp in Z direction
  //As we know +z is upwards. We cannot grasp any object from -z. Instead we rotate the z by its axis.
  //So we the grasp angle can be according to a1 or a2
  //We are not rotating every pose by its axis because if it is rotated then one finger will surely collide with z

  /*Eigen::Affine3d transformation_mat_inv_z =  create_transformation_matrix(0, 0,-sq.a3, 0,  -M_PI/2, 0);
  Eigen::Affine3d pose_in_center_inv_z = pose_in_center * transformation_mat_inv_z;
  Eigen::Affine3d back_to_place6 = pose_in_eigen * pose_in_center_inv_z ;
  Eigen::Affine3d back_to_place6_trnsformed = back_to_place6*transform_;
  geometry_msgs::Pose pose_inv_z;
  tf::poseEigenToMsg(back_to_place6_trnsformed, pose_inv_z);
  poses.push_back(pose_inv_z);*/

  grasp_execution::grasp gr_z_rot;
  geometry_msgs::Pose pose_z_rotated = rotatePose(pose_z, M_PI/2, 1,false);
  gr_z_rot.pose = pose_z_rotated;
  gr_z_rot.angle = sq.a1*10;
  grasps.push_back(gr_z_rot);
}

void CreateGrasps::filterGraspsByOpenningAngle(const sq_fitting::sq& sq, const std::vector<grasp_execution::grasp> &grasps_in, std::vector<grasp_execution::grasp> &grasps_out)
{
  for(int i=0;i<grasps_in.size();++i)
  {
    if(grasps_in[i].angle<ee_max_opening_angle_)
      grasps_out.push_back(grasps_in[i]);
  }
}

void CreateGrasps::filterGraspsByIK(const std::vector<grasp_execution::grasp> &grasps_in, std::vector<grasp_execution::grasp> &grasps_out)
{
  for(int i=0;i<grasps_in.size();++i)
  {
    geometry_msgs::PoseStamped q_stamped;
    q_stamped.header.frame_id = frame_id_ ;
    q_stamped.pose =grasps_in[i].pose;
    moveit_msgs::GetPositionIK srv;
    moveit_msgs::PositionIKRequest req;
    req.group_name = group_.getName();
    req.avoid_collisions = true;
    req.attempts = 10;
    req.timeout.fromSec(0.1);
    req.pose_stamped = q_stamped;
    std::cout<<q_stamped.pose.position.x<<" "<<q_stamped.pose.position.y<<" "<<q_stamped.pose.position.z<<std::endl;
    std::cout<<q_stamped.pose.orientation.x<<" "<<q_stamped.pose.orientation.y<<" "<<
                q_stamped.pose.orientation.z<<" "<<q_stamped.pose.orientation.w<<std::endl;
    srv.request.ik_request = req;
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    client.call(srv);
    /*if(client.call(srv))
    {
      std::cout<<"OK.Succes"<<std::endl;
      std::cout<<"Response is: "<<srv.response.error_code.val<<std::endl;
    }
    else
    {
      ROS_ERROR("Failed to call IK service");
      //return 1;
    }*/

    if (srv.response.error_code.val == 1)
        grasps_out.push_back(grasps_in[i]);

  }
}

void CreateGrasps::sample_initial_grasps()
{
  init_grasps_.header.frame_id = frame_id_;
  init_grasps_.header.stamp = ros::Time::now();
  for(int i=0;i<sqArr_ .sqs.size();++i)
    {
      std::vector<grasp_execution::grasp> grasps_vec_init;
      std::vector<grasp_execution::grasp> grasps_vec_filtered_by_angle;
      std::vector<grasp_execution::grasp> grasps_vec_filtered_by_ik;
      createInitGrasps(sqArr_ .sqs[i], grasps_vec_init);
      std::cout<<">>>>>>>>>>>>For object: "<<i+1<<"<<<<<<<<<<<<"<<std::endl;
      std::cout<<"After initial: "<<grasps_vec_init.size()<< "grasps"<<std::endl;
      filterGraspsByOpenningAngle(sqArr_ .sqs[i], grasps_vec_init, grasps_vec_filtered_by_angle);
      std::cout<<"After angle filtering: "<<grasps_vec_filtered_by_angle.size()<< "grasps"<<std::endl;
      filterGraspsByIK(grasps_vec_filtered_by_angle, grasps_vec_filtered_by_ik);
      std::cout<<"After Ik filtering: "<<grasps_vec_filtered_by_ik.size()<< "grasps"<<std::endl;
      for (int j=0;j< grasps_vec_filtered_by_ik.size();++j)
      {
        init_grasps_.grasps.push_back( grasps_vec_filtered_by_ik[j]);
      }
    }
}

void CreateGrasps::getGrasps(grasp_execution::graspArr &grasps)
{
  grasps = init_grasps_;
}
