#include <ros/ros.h>
#include <sq_grasp/segmentation.h>

const std::string CLOUD_TOPIC = "input_cloud";
const double ZMAX = 0;
const double ZMIN = 0;
static const int TH_POINTS = 400;

static const double VOXEL_RESOLUTION = 0.0075f;
static const double SEED_RESOLUTION = 0.03f;
static const double COLOR_IMPORTANCE = 0.0f;
static const double SPATIAL_IMPORTANCE = 1.0f;
static const double NORMAL_IMPORTANCE = 4.0f;

static bool USE_EXTENDED_CONVEXITY = false;
static bool USE_SANITY_CRITERION = true;
static const double CONCAVITY_TOLERANCE_THRESHOLD = 10;
static const double SMOOTHNESS_THRESHOLD = 0.1f;
static const int MIN_SEGMENT_SIZE = 3;


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "segmentation_node");
  ros::NodeHandle nh_("~");

  Segmentation::Parameters params;
  std::string cloud_topic;
  nh_.param("cloud_topic", cloud_topic, CLOUD_TOPIC);
  nh_.param("zmin", params.zmin_, ZMIN);
  nh_.param("zmax", params.zmax_, ZMAX);
  nh_.param("th_points", params.th_points_, TH_POINTS);
  nh_.param("voxel_resoluton", params.voxel_resolution_, VOXEL_RESOLUTION);
  nh_.param("seed_resolution", params.seed_resolution_, SEED_RESOLUTION);
  nh_.param("color_importance", params.color_importance_, COLOR_IMPORTANCE);
  nh_.param("spatial_importance", params.spatial_importance_, SPATIAL_IMPORTANCE);
  nh_.param("normal_importance", params.normal_importance_, NORMAL_IMPORTANCE);
  nh_.param("use_extended_convexity", params.use_extended_convexity_, USE_EXTENDED_CONVEXITY);
  nh_.param("use_sanity_criterion", params.use_sanity_criterion_, USE_SANITY_CRITERION);
  nh_.param("concavity_tolerance_threshold", params.concavity_tolerance_threshold_, CONCAVITY_TOLERANCE_THRESHOLD);
  nh_.param("smoothness_threshold", params.smoothness_threshold_, SMOOTHNESS_THRESHOLD);
  nh_.param("min_segment_size", params.min_segment_size_, MIN_SEGMENT_SIZE);


  std::cout<< "-----Segmentation Parameters------\n";
  std::cout<<" Input \n";
  std::cout<<" cloud_topic: "<<cloud_topic<<"\n";
  std::cout<<" zmin: "<<params.zmin_<<"\n";
  std::cout<<" zmax: "<<params.zmax_<<"\n";
  std::cout<<" th points: "<<params.th_points_<<"\n";
  std::cout<<" voxel resolution:"<<params.voxel_resolution_<<"\n";
  std::cout<<" seed resolution:"<<params.seed_resolution_<<"\n";
  std::cout<<" spatial importance:"<<params.spatial_importance_<<"\n";
  std::cout<<" color importance:"<<params.color_importance_<<"\n";
  std::cout<<" normal importace: "<<params.normal_importance_<<"\n";
  std::cout<<" use extended convexity: "<<params.use_extended_convexity_<<"\n";
  std::cout<<" use sanity criterion: "<<params.use_sanity_criterion_<<"\n";
  std::cout<<" concavity tolerance threshold: "<<params.concavity_tolerance_threshold_<<"\n";
  std::cout<<" smoothness threshold: "<< params.smoothness_threshold_<<"\n";
  std::cout<<" min segment size: "<<params.min_segment_size_<<"\n";


  Segmentation seg(nh_, cloud_topic, params);
  seg.segment();

  ros::spin();

}
