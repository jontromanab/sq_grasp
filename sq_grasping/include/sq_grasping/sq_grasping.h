#ifndef SQ_GRASPING_H
#define SQ_GRASPING_H

#include<ros/ros.h>
#include <iostream>
#include<sq_fitting/sqArray.h>
#include<grasp_execution/graspArr.h>
#include<sq_grasping/getGrasps.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PoseArray.h>
#include<visualization_msgs/MarkerArray.h>
#include<moveit/move_group_interface/move_group.h>
#include<visualization_msgs/InteractiveMarker.h>
#include<interactive_markers/interactive_marker_server.h>




class SQGrasping
{
public:
  SQGrasping(ros::NodeHandle& nh, const std::string& sq_topic, bool show_sq,
             bool show_grasp, const std::string output_frame, const std::string ee_group,
             const std::string ee_grasp_link, const std::string ee_joint, const std::string arm_group);
  ~SQGrasping();
  void runNode();

private:

  bool checkForTransform(const std::string& input_frame, const std::string& output_frame);
  void transformFrame(const std::string& input_frame, const std::string& output_frame,const geometry_msgs::Pose &pose_in, geometry_msgs::Pose& pose_out);
  void createGrasps(const sq_fitting::sqArray& sqs , grasp_execution::graspArr& gs);
  void createGraspsMarkers(const grasp_execution::graspArr& grasps, visualization_msgs::MarkerArray& markers);
  void sampleSQFromSQS(const sq_fitting::sqArray &sqs);
  bool serviceCallback(sq_grasping::getGrasps::Request& req, sq_grasping::getGrasps::Response& res);
  visualization_msgs::MarkerArray createGripperMarkerMoveIT(geometry_msgs::Pose pose, double opening_angle,int id);

  ros::ServiceServer service_;
  ros::ServiceClient client_;
  bool show_sq_;
  bool show_grasp_;


  grasp_execution::graspArr grasps_;
  sq_fitting::sqArray sqs_;
  ros::NodeHandle nh_;
  ros::Publisher superquadrics_pub_;
  ros::Publisher grasp_pub_;
  ros::Publisher grasp_pub_arrow_;
  sensor_msgs::PointCloud2 sq_cloud_;

  std::string output_frame_;
  std::string ee_grasp_link_;
  std::string arm_group_;
  std::string ee_name_;
  std::string ee_joint_;

  visualization_msgs::MarkerArray poses_arrow_;
  visualization_msgs::MarkerArray poses_;
  moveit::planning_interface::MoveGroup ee_group_;
  ros::AsyncSpinner spinner;



};




#endif // SQ_GRASPING_H
