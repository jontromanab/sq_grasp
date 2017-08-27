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




class SQGrasping
{
public:
  SQGrasping(ros::NodeHandle& nh, const std::string& sq_topic, bool show_sq,const std::string output_frame);
  ~SQGrasping();
  void runNode();

private:

  bool checkForTransform(const std::string& input_frame, const std::string& output_frame);
  void transformFrame(const std::string& input_frame, const std::string& output_frame,const geometry_msgs::Pose &pose_in, geometry_msgs::Pose& pose_out);
  void createGrasps(const sq_fitting::sqArray& sqs , grasp_execution::graspArr& gs);
  void createGraspsMarkers(const grasp_execution::graspArr& grasps, visualization_msgs::MarkerArray& markers);
  void sampleSQFromSQS(const sq_fitting::sqArray &sqs);
  bool serviceCallback(sq_grasping::getGrasps::Request& req, sq_grasping::getGrasps::Response& res);
  ros::ServiceServer service_;
  ros::ServiceClient client_;
  bool show_sq_;


  grasp_execution::graspArr grasps_;
  sq_fitting::sqArray sqs_;
  ros::NodeHandle nh_;
  ros::Publisher superquadrics_pub_;
  ros::Publisher grasp_pub_;
  sensor_msgs::PointCloud2 sq_cloud_;
  std::string output_frame_;
  visualization_msgs::MarkerArray poses_;
};




#endif // SQ_GRASPING_H
