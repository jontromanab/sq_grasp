#include <ros/ros.h>
#include<sq_grasping/create_grasps.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include<tf/transform_listener.h>
#include<sq_fitting/utils.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>

#include<moveit_msgs/GetPositionIK.h>
#include<moveit_msgs/PositionIKRequest.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

CreateGrasps::CreateGrasps(ros::NodeHandle &nh, sq_fitting::sqArray sqArr, geometry_msgs::Vector3 table_center,
                           const std::string group,const std::string ee_name,
                           double ee_max_opening_angle, double object_padding, double approach_value): nh_(nh),spinner(1), group_name_(group),
  ee_name_(ee_name), ee_max_opening_angle_(ee_max_opening_angle), object_padding_(object_padding),
  approach_value_(approach_value)
{
  sqArr_ = sqArr;
  table_center_ = table_center;
  init_grasps_.grasps.resize(0);
  frame_id_ = sqArr_.header.frame_id;
  group_.reset(new moveit::planning_interface::MoveGroup(group_name_));
  spinner.start();
  planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface());
  transform_ = createTransform(ee_name, group_->getEndEffectorLink());
  //client_= nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
}

CreateGrasps::~CreateGrasps()
{

}

void transformFrame(const std::string& input_frame, const std::string& output_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
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


bool CreateGrasps::findGraspFromSQ(const sq_fitting::sq &sq, grasp_execution::grasp& grasp)
{
  // 1.First we create Initial 6 grasps from major axes. +x, -x, +y, -y, +z, z_rot
  std::vector<grasp_execution::grasp> initial_grasps;
  createInitGrasps(sq, initial_grasps);
  std::cout<<"Initial grasps size: "<<initial_grasps.size()<<std::endl;
  //2. We filter grasps whose angles are more than max gripper openning angle
  std::vector<grasp_execution::grasp> grasps_filtered_by_angle;
  filterGraspsByOpenningAngle(initial_grasps, grasps_filtered_by_angle);
  if(grasps_filtered_by_angle.size()==0)
    return false;
  std::cout<<"Grasps filtered by angle size: "<<grasps_filtered_by_angle.size()<<std::endl;
  //3. We create approach poses for every grasp. If the original pose and approach pose both have IK solutions,
  // then only this grasp can proceed
  std::vector<grasp_execution::grasp> grasps_filtered_by_IK;
  grasps_filtered_by_IK.resize(0);

  //Creating approach direction for every IK filtered grasp
  for(int i=0;i<grasps_filtered_by_angle.size();++i)
  {
    if(isGraspReachable(grasps_filtered_by_angle[i]))
    {
      grasp_execution::grasp gr;
      gr = grasps_filtered_by_angle[i];
      geometry_msgs::Vector3 direction;
      findDirection(gr.pose,direction);
      gr.approach = direction;
      grasps_filtered_by_IK.push_back(gr);
    }
  }
  std::cout<<"Grasps filtered by IK: "<<grasps_filtered_by_IK.size()<<std::endl;
  //4. Filter the grasps based on their dist from object and current robot ee pose. Now we cut the barrier of choosing grasp in z direction or x/y direction
  if(grasps_filtered_by_IK.size() == 0)
    return false;
  else if (grasps_filtered_by_IK.size() == 1)
  {
    grasp = grasps_filtered_by_IK[0];
    return true;
  }
  else
  {
    grasp_execution::grasp gr;
    filterGraspByDistance(sq, grasps_filtered_by_IK, gr);
    grasp = gr;
    return true;
  }

}



double getDistanceBwPoses(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
  double x_dist  = pose1.position.x - pose2.position.x;
  double y_dist  = pose1.position.y - pose2.position.y;
  double z_dist  = pose1.position.z - pose2.position.z;
  double dist = sqrt((x_dist*x_dist)+(y_dist*y_dist)+(z_dist*z_dist));
  return dist;
}

void sq_create_transform(const geometry_msgs::Pose& pose, Eigen::Affine3f& transform)
{
  transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.translation()<<pose.position.x, pose.position.y, pose.position.z;
  transform.rotate(q);
}

Eigen::Affine3d CreateGrasps::createTransform(const std::string &grasp_frame, const std::string& planning_frame)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  Eigen::Affine3d transformation;
  //std::string planning_frame = group_->getEndEffectorLink();
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
    tf::poseMsgToEigen(inter_pose, transformation);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return transformation;
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

void CreateGrasps::findApproachPose(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out)
{
  Eigen::Affine3d pose_in_eigen;
  tf::poseMsgToEigen(pose_in, pose_in_eigen);
  Eigen::Affine3d pose_inv = pose_in_eigen.inverse();
  Eigen::Affine3d pose_in_center = pose_in_eigen * pose_inv;

  Eigen::Affine3d translation_matrix(Eigen::Translation3d(Eigen::Vector3d(-approach_value_, 0, 0)));
  Eigen::Affine3d translated_approach = translation_matrix * pose_in_center;
  Eigen::Affine3d back_to_world = pose_in_eigen * translated_approach;
  tf::poseEigenToMsg(back_to_world, pose_out);
}

void CreateGrasps::findDirectionVectorfromApproachPose(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, geometry_msgs::Vector3 &direction)
{
  direction.x = pose2.position.x - pose1.position.x;
  direction.y = pose2.position.y - pose1.position.y;
  direction.z = pose2.position.z - pose1.position.z;
}

void CreateGrasps::findDirection(const geometry_msgs::Pose &pose, geometry_msgs::Vector3 &dir)
{
  geometry_msgs::Pose approach_pose;
  findApproachPose(pose, approach_pose);
  findDirectionVectorfromApproachPose(pose, approach_pose, dir);
}

void CreateGrasps::findApproachPoseFromDir(const geometry_msgs::Pose &pose_in, const geometry_msgs::Vector3 &dir, geometry_msgs::Pose &pose_out)
{
  pose_out.orientation = pose_in.orientation;
  pose_out.position.x = pose_in.position.x + dir.x;
  pose_out.position.y = pose_in.position.y + dir.y;
  pose_out.position.z = pose_in.position.z + dir.z;
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
  //We are not rotating every pose by its axis because if it is rotated then one finger will surely collide with table

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

void CreateGrasps::filterGraspsByOpenningAngle(const std::vector<grasp_execution::grasp> &grasps_in, std::vector<grasp_execution::grasp> &grasps_out)
{
  for(int i=0;i<grasps_in.size();++i)
  {
    if(grasps_in[i].angle<ee_max_opening_angle_)
      grasps_out.push_back(grasps_in[i]);
  }
}

bool CreateGrasps::isGraspReachable(const grasp_execution::grasp grasp)
{
  std::vector<geometry_msgs::Pose> poses;
  poses.push_back(grasp.pose);
  geometry_msgs::Pose approach_pose;
  findApproachPose(grasp.pose, approach_pose);
  poses.push_back(approach_pose);
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  std::vector<bool> results(2, false);
  for (int i=0;i<poses.size();++i)
  {
    geometry_msgs::PoseStamped q_stamped;
    q_stamped.header.frame_id = frame_id_ ;
    geometry_msgs::Pose trans_pose;
    transformFrame(group_->getPlanningFrame(),frame_id_,poses[i], trans_pose);
    q_stamped.pose = trans_pose;
    moveit_msgs::GetPositionIK srv;
    moveit_msgs::PositionIKRequest req;
    req.group_name = group_->getName();
    req.avoid_collisions = true;
    req.attempts = 10;
    req.timeout.fromSec(0.1);
    req.pose_stamped = q_stamped;
    srv.request.ik_request = req;
    client.call(srv);
    if (srv.response.error_code.val == 1)
      results[i] = true;
   }
 if (std::all_of(std::begin(results),std::end(results),[](bool i){return i;})){return true;}
 else
   return false;
}

bool CreateGrasps::filterGraspsByIK(const std::vector<grasp_execution::grasp> &grasps_in, std::vector<grasp_execution::grasp> &grasps_out)
{
  grasps_out.resize(0);
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  std::vector<geometry_msgs::Pose> poses;
  for (int i=0;i<grasps_in.size();++i)
  {
    poses.push_back(grasps_in[i].pose);
    geometry_msgs::Pose approach_pose;
    findApproachPose(grasps_in[i].pose, approach_pose);
    poses.push_back(approach_pose);
  }

  for(int i=0;i<poses.size();i+=2)
  {
    std::cout<<"Loop "<<i<<std::endl;
    geometry_msgs::PoseStamped q_stamped;
    q_stamped.header.frame_id = frame_id_ ;
    geometry_msgs::Pose trans_pose;
    transformFrame(group_->getPlanningFrame(),frame_id_,poses[i], trans_pose);
    q_stamped.pose = trans_pose;
    moveit_msgs::GetPositionIK srv;
    moveit_msgs::PositionIKRequest req;
    req.group_name = group_->getName();
    req.avoid_collisions = true;
    req.attempts = 10;
    req.timeout.fromSec(0.1);
    req.pose_stamped = q_stamped;

    geometry_msgs::PoseStamped q_stamped1;
    q_stamped1.header.frame_id = frame_id_ ;
    geometry_msgs::Pose trans_pose1;
    transformFrame(group_->getPlanningFrame(),frame_id_,poses[i+1], trans_pose1);
    q_stamped1.pose = trans_pose1;
    moveit_msgs::GetPositionIK srv1;
    moveit_msgs::PositionIKRequest req1;
    req1.group_name = group_->getName();
    req1.avoid_collisions = true;
    req1.attempts = 10;
    req1.timeout.fromSec(0.1);
    req1.pose_stamped = q_stamped1;
       
    //std::cout<<trans_pose.position.x<<" "<<trans_pose.position.y<<" "<<trans_pose.position.z<<std::endl;
    //std::cout<<trans_pose.orientation.x<<" "<<trans_pose.orientation.y<<" "<<
               // trans_pose.orientation.z<<" "<<trans_pose.orientation.w<<std::endl;
    srv.request.ik_request = req;
    srv1.request.ik_request = req1;

    client.call(srv);
    client.call(srv1);
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

    if (srv.response.error_code.val == 1 && srv1.response.error_code.val==1)
        grasps_out.push_back(grasps_in[i]);

  }
  if(grasps_out.size()>0)
    return true;
  else
    return false;
}

double CreateGrasps::getDistanceFromRobot(geometry_msgs::Pose& pose)
{
  Eigen::Affine3d transform = createTransform(group_->getPlanningFrame(),group_->getEndEffectorLink());
  geometry_msgs::Pose new_pose;
  tf::poseEigenToMsg(transform, new_pose);
  double dist =  getDistanceBwPoses(new_pose, pose);
  return dist;
}

void CreateGrasps::filterGraspByDistance(const sq_fitting::sq& sq, const std::vector<grasp_execution::grasp>& grasps_in,
                             grasp_execution::grasp& grasp_out)
{
  double dist = 100.0;
  grasp_execution::grasp gr;
  for(int i=0;i<grasps_in.size();++i)
  {
    geometry_msgs::Pose approach_pose;
    geometry_msgs::Pose trans_pose;
    findApproachPoseFromDir(grasps_in[i].pose, grasps_in[i].approach, approach_pose);
    transformFrame(frame_id_, group_->getPlanningFrame(),approach_pose, trans_pose);
    double dist_from_ee = getDistanceFromRobot(trans_pose);
    double dist_from_center = getDistanceBwPoses(approach_pose, sq.pose);
    double angle_value = grasps_in[i].angle;
    double new_dist = dist_from_ee + dist_from_center+angle_value;
    if(new_dist<dist)
    {
      dist = new_dist;
      gr = grasps_in[i];
     }
   }
  grasp_out = gr;
}

void CreateGrasps::sample_grasps()
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id_;
  collision_object.id = "table";

  /* Define a table to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = 0.005;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  table_center_.x;
  box_pose.position.y = table_center_.y;
  box_pose.position.z =  table_center_.z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  std::vector<std::string> objects;

  //Make all the superquadrics collision objects
  for(int i=0;i<sqArr_.sqs.size();++i)
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id_;
    std::string obj = "obj";
    obj+=std::to_string(i);
    collision_object.id = obj;
    objects.push_back(obj);

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2*sqArr_.sqs[i].a1 + object_padding_;
    primitive.dimensions[1] = 2*sqArr_.sqs[i].a2 + object_padding_;
    primitive.dimensions[2] = 2*sqArr_.sqs[i].a3 + object_padding_;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.position =  sqArr_.sqs[i].pose.position;
    box_pose.orientation =  sqArr_.sqs[i].pose.orientation;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
  }

  // Now, let's add the collision object into the world
  ROS_INFO("Add table into the world");
  planning_scene_interface_->addCollisionObjects(collision_objects);
  sleep(1.0);

  
  init_grasps_.header.frame_id = frame_id_;
  init_grasps_.header.stamp = ros::Time::now();
  init_grasps_.table_center = table_center_;
  for(int i=0;i<sqArr_ .sqs.size();++i)
    {
      std::cout<<">>>>>>>>>>>>For object: "<<i+1<<" <<<<<<<<<<<<"<<std::endl;
      grasp_execution::grasp gr;
      if(findGraspFromSQ(sqArr_.sqs[i], gr))
      {
        init_grasps_.grasps.push_back(gr);
      }
    }

  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the collision objects from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  for (int i=0;i<objects.size();++i)
  {
    object_ids.push_back(objects[i]);
  }
  planning_scene_interface_->removeCollisionObjects(object_ids);
  sleep(1.0);
}

void CreateGrasps::getGrasps(grasp_execution::graspArr &grasps)
{
  grasps = init_grasps_;
}
