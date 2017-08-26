#ifndef SQ_GRASPING_H
#define SQ_GRASPING_H

#include<ros/ros.h>
#include <iostream>
#include<sq_fitting/sqArray.h>
#include<grasp_execution/graspArr.h>
#include<sq_grasping/getGrasps.h>
#include<sensor_msgs/PointCloud2.h>



class SQGrasping
{
public:
  SQGrasping(ros::NodeHandle& nh, const std::string& sq_topic, bool show_sq);
  ~SQGrasping();
  void runNode();

private:

  void sampleSQFromSQS(const sq_fitting::sqArray &sqs);
  bool serviceCallback(sq_grasping::getGrasps::Request& req, sq_grasping::getGrasps::Response& res);
  ros::ServiceServer service_;
  ros::ServiceClient client_;
  bool show_sq_;


  grasp_execution::graspArr grasps_;
  sq_fitting::sqArray sqs_;
  ros::NodeHandle nh_;
  ros::Publisher superquadrics_pub_;
  sensor_msgs::PointCloud2 sq_cloud_;
};




#endif // SQ_GRASPING_H
