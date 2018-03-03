#ifndef SQ_FITTER_H
#define SQ_FITTER_H
#include <ros/ros.h>
#include <sq_fitting/segmentation.h>
#include <sq_fitting/fitting.h>
#include <sq_fitting/sampling.h>
#include <sq_fitting/sq.h>
#include <sq_fitting/sqArray.h>
#include <sq_fitting/get_sq.h>
#include <pcl/filters/filter.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <mutex>
#include <thread>

typedef std::vector<std::pair<sq_fitting::sq, CloudPtr> > ParamMultiVector;

/**
 * @brief Class for fitting superquadrics from segmented objects and samplng
 * Additionally this class also provide a ROS server which returns parameters
 * for superquadrics
 */

class SQFitter
{
public:
  /**
   * @brief The Parameters struct contains parameters for this class
   */
  struct Parameters
  {
    std::vector<double> ws_limits;
    bool remove_nan;
    std::string pose_est_method;
  };

  /**
   * @brief Constructor
   * @param node Nodehandle
   * @param segmentation_service name of the segmentation service
   * @param cloud_topic topic name of the cloud from inout device
   * @param output_frame the output frame to which everything will be transformed
   * @param params struct Parameters
   */
  SQFitter(ros::NodeHandle &node, const std::string &segmentation_service,
           const std::string &cloud_topic, const std::string &output_frame, const SQFitter::Parameters &params);

  /**
   * @brief Destructor
   */
  ~SQFitter();

  /**
   * @brief public class to run the node and publish clouds
   */
  void fit();


private:
  /**
   * @brief callback function to segmentation, fitting and sampling
   * @param input cloud from input device
   */
  void cloud_callback(const sensor_msgs::PointCloud2& input);

  /**
   * @brief filteres the workspace by ws_limits params
   * @param cloud input cloud
   * @param filtered_cloud filtered cloud
   */
  void filterWorkSpace(const CloudPtr& cloud, CloudPtr& filtered_cloud);

  /**
   * @brief Obtains segmented objects and stores in Objects_ vector
   * @param cloud input cloud
   */
  void getSegmentedObjects(CloudPtr& cloud);

  /**
   * @brief publishes all the clouds on corresponding ROS topics
   */
  void publishClouds();

  /**
   * @brief transforms frames to output frame
   * @param pose_in geometry_msgs::Pose
   * @param pose_out geometry_msgs::Pose
   */
  void transformFrame(const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);

  /**
   * @brief transforms cloud to output_frame
   * @param cloud_in
   * @param cloud_out
   */
  void transformFrameCloud(const CloudPtr& cloud_in, CloudPtr& cloud_out);

  /**
   * @brief transform cloud back to sensor frame
   * @param cloud_in
   * @param cloud_out
   */
  void transformFrameCloudBack(const CloudPtr& cloud_in, CloudPtr& cloud_out);

  /**
   * @brief statistical outlier removal filter
   * @param cloud_in
   * @param cloud_out
   */
  void filter_StatOutlier(CloudPtr& cloud_in, CloudPtr& cloud_out);

  /**
   * @brief radius outlier removal filter
   * @param cloud_in
   * @param cloud_out
   */
  void filter_RadiusOutlier(CloudPtr& cloud_in, CloudPtr& cloud_out);

  /**
   * @brief mirrors the cloud
   * @param cloud_in
   * @param cloud_out
   */
  void mirror_cloud(CloudPtr& cloud_in, CloudPtr& cloud_out);

  /**
   * @brief threaded function to fit and sample each segmented object and store them in a
   * vector
   * @param cloud_in individual object cloud
   * @param method pca/iteration
   * @param pvector
   */
  void fitAndSampleTh(CloudPtr &cloud_in,std::string& method, ParamMultiVector& pvector);

  /**
   * @brief function to fit and sample segmented objects
   * @param objs segmented objects(cloudPtr)
   * @param pvector vector to store mapping between param and its cloud
   */
  void fitAndSample(std::vector<CloudPtr>& objs, ParamMultiVector& pvector);

  /**
   * @brief serviceCallback to obtain SQ parameters
   * @param req bool
   * @param res SQ_ARRAY
   * @return
   */
  bool serviceCallback(sq_fitting::get_sq::Request &req, sq_fitting::get_sq::Response &res);

  ///serviceClinet to segmentation server
  ros::ServiceClient client_;

  ///serviceServec to get_sq
  ros::ServiceServer service_;

  //ROS clouds for visualization
  ///table cloud
  sensor_msgs::PointCloud2 table_cloud_;
  ///objects cloud
  sensor_msgs::PointCloud2 objects_cloud_ros_;
  ///superquadrics clouds
  sensor_msgs::PointCloud2 sq_cloud_;
  ///workspace filtered clouds
  sensor_msgs::PointCloud2 ws_filtered_cloud_ros_;
  ///Input msg
  sensor_msgs::PointCloud2 input_msg_;
  ///mirrored cloud
  sensor_msgs::PointCloud2 cut_cloud_ros_;

  //Internal PointClouds
  ///input pointcloud
  CloudPtr cloud_;
  ///filtered pointcloud
  CloudPtr filtered_cloud_;
  ///Mirrored cloud
  CloudPtr cut_cloud_;

  //ROS subscribers and Publishers
  ///Cloud subscriber
  ros::Subscriber cloud_sub_;
  ///table cloud publisher
  ros::Publisher table_pub_;
  ///segmented objects cloud publisher
  ros::Publisher objects_pub_;
  ///superquadrics cloud publisher
  ros::Publisher superquadrics_pub_;
  ///ws filtered cloud publisher
  ros::Publisher filtered_cloud_pub_;
  ///superquadrics poses publisher
  ros::Publisher poses_pub_;
  ///mirrored cloud publisher
  ros::Publisher cut_cloud_pub_;

  //Internal containers
  ///Vector to store segmented object clouds
  std::vector<CloudPtr> Objects_;
  ///multivector to store mapping between SQ param and cloudPtr
  ParamMultiVector pVector_;
  ///Array of SQ poses
  geometry_msgs::PoseArray poseArr_;
  ///parameters for SQ fitting
  SQFitter::Parameters sq_param_;
  ///container for superquadrics params
  sq_fitting::sqArray sqArr_;

  ///Node running
  bool initialized;
  ///name of outout frame
  std::string output_frame_;
  ///table center
  geometry_msgs::Vector3 table_center_;
  ///nodehandle
  ros::NodeHandle nh_;
  ///mutex to lock fitting and sampling threads
  std::mutex mu_;
};

#endif // SQ_FITTER_H
