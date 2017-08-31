#include<sq_grasping/create_grasps.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include<tf/transform_listener.h>

CreateGrasps::CreateGrasps(sq_fitting::sqArray sqArr, const std::string group,const std::string ee_name):
  group_(group), ee_name_(ee_name)
{
  sqArr_ = sqArr;
  init_grasps_.grasps.resize(0);
  frame_id_ = sqArr_.header.frame_id;

}

CreateGrasps::~CreateGrasps()
{

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

void CreateGrasps::createGraspPoses(const sq_fitting::sq &sq, std::vector<geometry_msgs::Pose> &poses)
{
  //First transform the pose to the origin
  Eigen::Affine3d pose_in_eigen;
  tf::poseMsgToEigen(sq.pose, pose_in_eigen);
  Eigen::Affine3d pose_inv = pose_in_eigen.inverse();
  Eigen::Affine3d pose_in_center = pose_in_eigen * pose_inv;

  //First grasp in x direction
  Eigen::Affine3d translation_matrix_orig(Eigen::Translation3d(Eigen::Vector3d(-sq.a1, 0, 0)));
  Eigen::Affine3d pose_in_center_trnsl1 = translation_matrix_orig* pose_in_center;
  Eigen::Affine3d back_to_place1 = pose_in_eigen * pose_in_center_trnsl1 ;
  geometry_msgs::Pose orig_pose;
  tf::poseEigenToMsg(back_to_place1, orig_pose);
  poses.push_back(orig_pose);


  //Second grasp in x direction
  /*Eigen::Affine3d transformation_mat_x =  create_transformation_matrix(sq.a1, 0, 0, 0, 0, M_PI);
  Eigen::Affine3d pose_in_center_inv_x = pose_in_center * transformation_mat_x;
  Eigen::Affine3d back_to_place2 = pose_in_eigen * pose_in_center_inv_x ;
  geometry_msgs::Pose inverted_pose_x;
  tf::poseEigenToMsg(back_to_place2, inverted_pose_x);
  poses.push_back(inverted_pose_x);

  //First grasp in Y direction
  Eigen::Affine3d transformation_mat_y =  create_transformation_matrix(0, -sq.a2, 0, 0, 0 , M_PI/2);
  Eigen::Affine3d pose_in_center_y = pose_in_center * transformation_mat_y;
  Eigen::Affine3d back_to_place3 = pose_in_eigen * pose_in_center_y ;
  geometry_msgs::Pose pose_y;
  tf::poseEigenToMsg(back_to_place3, pose_y);
  poses.push_back(pose_y);


  //Second Grasp in Y direction
  Eigen::Affine3d transformation_mat_inv_y =  create_transformation_matrix(0, sq.a2, 0, 0, 0 , -M_PI/2);
  Eigen::Affine3d pose_in_center_inv_y = pose_in_center * transformation_mat_inv_y;
  Eigen::Affine3d back_to_place4 = pose_in_eigen * pose_in_center_inv_y ;
  geometry_msgs::Pose pose_inv_y;
  tf::poseEigenToMsg(back_to_place4, pose_inv_y);
  poses.push_back(pose_inv_y);

  //First grasp in Z direction
  Eigen::Affine3d transformation_mat_z =  create_transformation_matrix(0, 0,sq.a3, 0,  M_PI/2, 0);
  Eigen::Affine3d pose_in_center_z = pose_in_center * transformation_mat_z;
  Eigen::Affine3d back_to_place5 = pose_in_eigen * pose_in_center_z ;
  geometry_msgs::Pose pose_z;
  tf::poseEigenToMsg(back_to_place5, pose_z);
  poses.push_back(pose_z);

  //Second grasp in Z direction
  Eigen::Affine3d transformation_mat_inv_z =  create_transformation_matrix(0, 0,-sq.a3, 0,  -M_PI/2, 0);
  Eigen::Affine3d pose_in_center_inv_z = pose_in_center * transformation_mat_inv_z;
  Eigen::Affine3d back_to_place6 = pose_in_eigen * pose_in_center_inv_z ;
  geometry_msgs::Pose pose_inv_z;
  tf::poseEigenToMsg(back_to_place6, pose_inv_z);
  poses.push_back(pose_inv_z);*/

}

void CreateGrasps::sample_initial_grasps()
{
  init_grasps_.header.frame_id = frame_id_;
  init_grasps_.header.stamp = ros::Time::now();
  for(int i=0;i<sqArr_ .sqs.size();++i)
    {
      std::vector<geometry_msgs::Pose> poses;
      createGraspPoses(sqArr_ .sqs[i], poses);
      for (int j=0;j<poses.size();++j)
      {
        grasp_execution::grasp gr;
        gr.pose = poses[j];
        init_grasps_.grasps.push_back(gr);
      }
      /*gr.pose = sqArr_ .sqs[i].pose;
      gr.pose.position.z = sqArr_.sqs[i].pose.position.z + 0.14;
      gr.pose.orientation.x = 0.639;
      gr.pose.orientation.y = -0.326;
      gr.pose.orientation.z = -0.616;
      gr.pose.orientation.w = -0.327;
      gr.angle = 0.79;
      gr.approach.x = 0.0;
      gr.approach.y = 0.0;
      gr.approach.z = -1.0;*/

    }


}

void CreateGrasps::getGrasps(grasp_execution::graspArr &grasps)
{
  grasps = init_grasps_;
}
