#ifndef CREATE_GRASPS_H
#define CREATE_GRASPS_H

#include<sq_fitting/sqArray.h>
#include<grasp_execution/graspArr.h>
#include<geometry_msgs/Pose.h>
#include<moveit/move_group_interface/move_group.h>

#include<tf/transform_listener.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

class CreateGrasps
{
public:
  CreateGrasps(sq_fitting::sqArray sqArr, const std::string group,
               const std::string ee_name, double ee_max_opening_angle);
  ~CreateGrasps();
  void sample_initial_grasps();
  void getGrasps(grasp_execution::graspArr &grasps);
private:
  sq_fitting::sqArray sqArr_;
  grasp_execution::graspArr init_grasps_;
  std::string frame_id_;
  std::string ee_name_;

  Eigen::Affine3d transform_;


  void createInitGrasps(const sq_fitting::sq& sq,
                        std::vector<grasp_execution::grasp>& grasps);

  void filterGraspsByOpenningAngle(const sq_fitting::sq& sq,const std::vector<grasp_execution::grasp>& grasps_in,
                                   std::vector<grasp_execution::grasp>& grasps_out);

  void createTransform(const std::string& grasp_frame);
  void TransformPose(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);

  moveit::planning_interface::MoveGroup group_;
  double ee_max_opening_angle_;



};

#endif // CREATE_GRASPS_H
