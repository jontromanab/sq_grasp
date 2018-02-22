#include<ros/ros.h>
#include<sq_fitting/sq_fitter.h>

const std::string CLOUD_TOPIC = "input_cloud";
//const std::string output_frame = "\world";
static const bool REMOVE_NAN = true;
const double WORKSPACE[6] = {-0.23, 0.23, -0.35, 0.07, 0.95, 2.5};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "super");
  ros::NodeHandle nh_("~");
  SQFitter::Parameters params;
  std::string cloud_topic, output_frame, segmentation_service;
  nh_.getParam("cloud_topic", cloud_topic);
  nh_.getParam("output_frame", output_frame);
  nh_.getParam("workspace", params.ws_limits);
  nh_.getParam("pose_est_method", params.pose_est_method);
  nh_.param("remove_nan", params.remove_nan, REMOVE_NAN);
  nh_.getParam("segmentation_service", segmentation_service);

  SQFitter sqfit(nh_, segmentation_service, cloud_topic, output_frame, params);
  sqfit.fit();
  ros::spin();
  return 0;
}
