#ifndef SQ_FITTER_H
#define SQ_FITTER_H
#include<ros/ros.h>
#include<sq_fitting/segmentation.h>
#include<sq_fitting/fitting.h>
#include<sq_fitting/sampling.h>
#include<sq_fitting/sq.h>
#include<sq_fitting/sqArray.h>
#include<sq_fitting/get_sq.h>
#include <pcl/filters/filter.h>
#include <geometry_msgs/PoseArray.h>
#include<visualization_msgs/Marker.h>

class SQFitter
{
public:
  struct Parameters
  {
    //Segmentation::Parameters seg_params;
    std::vector<double> ws_limits;
    bool remove_nan;
    std::string pose_est_method;
  };

  SQFitter(ros::NodeHandle &node, const std::string &cloud_topic, const std::string &output_frame, const SQFitter::Parameters &params);
  void fit();


private:
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input);
  void filterCloud(const CloudPtr& cloud, CloudPtr& filtered_cloud);
  //void filterCloud2(const CloudPtr& cloud, CloudPtr& filtered_cloud);
  void getSegmentedObjects(CloudPtr& cloud);
  void getSegmentedObjectsIRI(CloudPtr& cloud);
  void getSuperquadricParameters(std::vector<sq_fitting::sq>& params);
  void sampleSuperquadrics(const std::vector<sq_fitting::sq>& params);
  void publishClouds();
  void transformFrame(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);
  void transformFrameCloud(const CloudPtr& cloud_in, CloudPtr& cloud_out);
  void transformFrameCloudBack(const CloudPtr& cloud_in, CloudPtr& cloud_out);

  void filter_StatOutlier(CloudPtr& cloud_in, CloudPtr& cloud_out);
  void filter_RadiusOutlier(CloudPtr& cloud_in, CloudPtr& cloud_out);

  void mirror_cloud(CloudPtr& cloud_in, CloudPtr& cloud_out);
  void createCenterMarker(const double x, const double y, const double z);


  ros::ServiceClient client_;
  ros::ServiceClient client_param;

  bool serviceCallback(sq_fitting::get_sq::Request &req, sq_fitting::get_sq::Response &res);


  sensor_msgs::PointCloud2 table_cloud_;
  sensor_msgs::PointCloud2 objects_on_table_cloud_;
  sensor_msgs::PointCloud2 objects_cloud_;
  sensor_msgs::PointCloud2 sq_cloud_;
  sensor_msgs::PointCloud2 filtered_cloud_ros_;
  sensor_msgs::PointCloud2 input_msg_;
  sensor_msgs::PointCloud2 cut_cloud_ros_;
  sensor_msgs::PointCloud2 transformed_cloud_ros_;

  CloudPtr cloud_;
  CloudPtr transformed_cloud_;
  CloudPtr filtered_cloud_;
  CloudPtr table_plane_cloud_;
  CloudPtr segmented_objects_cloud_;
  CloudPtr objects_on_table_;
  CloudPtr cut_cloud_;




  std::vector<CloudPtr> Objects_;
  std::vector<sq_fitting::sq> params_;
  geometry_msgs::PoseArray poseArr_;

  ros::Subscriber cloud_sub_;
  ros::Publisher table_pub_;
  ros::Publisher objects_on_table_pub_;
  ros::Publisher objects_pub_;
  ros::Publisher superquadrics_pub_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher poses_pub_;
  ros::Publisher cut_cloud_pub_;
  ros::Publisher sqs_pub_;
  ros::Publisher center_pub_;
  ros::Publisher transformed_pub_;

  visualization_msgs::Marker centerPoint_;


  SQFitter::Parameters sq_param_;
  //Segmentation::Parameters seg_param_;
  sq_fitting::sqArray sqArr_;
  bool initialized;
  std::string output_frame_;
  ros::ServiceServer service_;
  geometry_msgs::Vector3 table_center_;

};

#endif // SQ_FITTER_H
