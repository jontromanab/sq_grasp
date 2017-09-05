#ifndef CREATE_GRASPS_H
#define CREATE_GRASPS_H

#include<sq_fitting/sqArray.h>
#include<grasp_execution/graspArr.h>
#include<geometry_msgs/Pose.h>
#include<moveit/move_group_interface/move_group.h>
#include<sensor_msgs/PointCloud2.h>

#include<tf/transform_listener.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include<moveit_msgs/PlanningScene.h>
#include<moveit_msgs/ApplyPlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


class CreateGrasps
{
public:
  CreateGrasps(ros::NodeHandle &nh, sq_fitting::sqArray sqArr, geometry_msgs::Vector3 table_center,
               const std::string group, const std::string ee_name, double ee_max_opening_angle,
               double object_padding, double approach_value);
  ~CreateGrasps();
  void sample_grasps();
  void getGrasps(grasp_execution::graspArr &grasps);
private:
  ros::NodeHandle nh_;
  sq_fitting::sqArray sqArr_;
  grasp_execution::graspArr init_grasps_;
  std::string frame_id_;
  std::string ee_name_;

  Eigen::Affine3d transform_;

  bool findGraspFromSQ(const sq_fitting::sq& sq,grasp_execution::grasp& grasps);

  void findApproachPose(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);

  void findApproachPoseFromDir(const geometry_msgs::Pose& pose_in, const geometry_msgs::Vector3& dir,
                               geometry_msgs::Pose& pose_out);

  void findDirectionVectorfromApproachPose(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2,
                           geometry_msgs::Vector3& direction);

  void findDirection(const geometry_msgs::Pose& pose, geometry_msgs::Vector3& dir);

  void createInitGrasps(const sq_fitting::sq& sq,
                        std::vector<grasp_execution::grasp>& grasps);

  void filterGraspsByOpenningAngle(const std::vector<grasp_execution::grasp>& grasps_in,
                                   std::vector<grasp_execution::grasp>& grasps_out);

  bool filterGraspsByIK(const std::vector<grasp_execution::grasp>& grasps_in,
                        std::vector<grasp_execution::grasp>& grasps_out);

  void getClosestToCenterGrasp(const std::vector<grasp_execution::grasp>& grasps_in, const sq_fitting::sq& sq,
                               grasp_execution::grasp& final_grasp);

  double getDistanceFromRobot(geometry_msgs::Pose& pose);

  Eigen::Affine3d createTransform(const std::string &grasp_frame, const std::string& planning_frame);

  void TransformPose(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);

  bool isGraspReachable(const grasp_execution::grasp grasp);
  //void getTable(const sensor_msgs::PointCloud2ConstPtr &table_msg);

  std::string group_name_;
  boost::scoped_ptr<moveit::planning_interface::MoveGroup> group_;
  double ee_max_opening_angle_;
  double object_padding_;
  double approach_value_;
  ros::ServiceClient client_;
  ros::AsyncSpinner spinner;
  boost::scoped_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  geometry_msgs::Vector3 table_center_;


};

#endif // CREATE_GRASPS_H
