#include<ros/ros.h>
#include<sq_fitting/sq_fitter.h>

const std::string CLOUD_TOPIC = "input_cloud";
//const std::string output_frame = "\world";
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

static const bool REMOVE_NAN = true;
const double WORKSPACE[6] = {-0.23, 0.23, -0.35, 0.07, 0.95, 2.5};



int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "super");
  ros::NodeHandle nh_("~");

  SQFitter::Parameters params;
  std::string cloud_topic, output_frame;
  nh_.getParam("cloud_topic", cloud_topic);
  nh_.getParam("output_frame", output_frame);
  nh_.getParam("workspace", params.ws_limits);
  nh_.param("remove_nan", params.remove_nan, REMOVE_NAN);
  nh_.param("zmin", params.seg_params.zmin, ZMIN);
  nh_.param("zmax", params.seg_params.zmax, ZMAX);
  nh_.param("th_points", params.seg_params.th_points, TH_POINTS);
  nh_.param("voxel_resoluton", params.seg_params.voxel_resolution, VOXEL_RESOLUTION);
  nh_.param("seed_resolution", params.seg_params.seed_resolution, SEED_RESOLUTION);
  nh_.param("color_importance", params.seg_params.color_importance, COLOR_IMPORTANCE);
  nh_.param("spatial_importance", params.seg_params.spatial_importance, SPATIAL_IMPORTANCE);
  nh_.param("normal_importance", params.seg_params.normal_importance, NORMAL_IMPORTANCE);
  nh_.param("use_extended_convexity", params.seg_params.use_extended_convexity, USE_EXTENDED_CONVEXITY);
  nh_.param("use_sanity_criterion", params.seg_params.use_sanity_criterion, USE_SANITY_CRITERION);
  nh_.param("concavity_tolerance_threshold", params.seg_params.concavity_tolerance_threshold, CONCAVITY_TOLERANCE_THRESHOLD);
  nh_.param("smoothness_threshold", params.seg_params.smoothness_threshold, SMOOTHNESS_THRESHOLD);
  nh_.param("min_segment_size", params.seg_params.min_segment_size, MIN_SEGMENT_SIZE);





  SQFitter sqfit(nh_, cloud_topic, output_frame,params);
  /*std::cout<< "-----Segmentation Parameters------\n";
  std::cout<<" Input \n";
  std::cout<<" cloud_topic: "<<cloud_topic<<"\n";
  std::cout<<"output_frame: "<<output_frame<<"\n";
  std::cout<<" zmin: "<<params.seg_params.zmin<<"\n";
  std::cout<<" zmax: "<<params.seg_params.zmax<<"\n";
  std::cout<<" th points: "<<params.seg_params.th_points<<"\n";
  std::cout<<" voxel resolution:"<<params.seg_params.voxel_resolution<<"\n";
  std::cout<<" seed resolution:"<<params.seg_params.seed_resolution<<"\n";
  std::cout<<" spatial importance:"<<params.seg_params.spatial_importance<<"\n";
  std::cout<<" color importance:"<<params.seg_params.color_importance<<"\n";
  std::cout<<" normal importace: "<<params.seg_params.normal_importance<<"\n";
  std::cout<<" use extended convexity: "<<params.seg_params.use_extended_convexity<<"\n";
  std::cout<<" use sanity criterion: "<<params.seg_params.use_sanity_criterion<<"\n";
  std::cout<<" concavity tolerance threshold: "<<params.seg_params.concavity_tolerance_threshold<<"\n";
  std::cout<<" smoothness threshold: "<< params.seg_params.smoothness_threshold<<"\n";
  std::cout<<" min segment size: "<<params.seg_params.min_segment_size<<"\n";*/
  sqfit.fit();


  ros::spin();

}
