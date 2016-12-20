#include <sq_grasp/segmentation.h>


Segmentation::Segmentation(ros::NodeHandle &node, const std::string &cloud_topic, const Parameters &params):
  cloud(new pcl::PointCloud<PointT>), table_plane_cloud(new pcl::PointCloud<PointT>)
{

    //Ros subscriber for the input cloud
    cloud_sub_ = node.subscribe(cloud_topic, 1, &Segmentation::cloud_callback, this);
    cloud_pub_ = node.advertise<sensor_msgs::PointCloud2> ("tabletop_object",10);
    table_pub_ = node.advertise<sensor_msgs::PointCloud2> ("table",10);
    convex_hull_pub_ = node.advertise<sensor_msgs::PointCloud2> ("convexHull",10);
     segmented_objects_pub_= node.advertise<sensor_msgs::PointCloud2> ("objects",10);

    //Initializing the Parameters
    this->zmin_ = params.zmin_;
    this->zmax_ = params.zmax_;
    this->th_points_ = params.th_points_;

    this->voxel_resolution_ = params.voxel_resolution_;
    this->seed_resolution_ = params.seed_resolution_;
    this->color_importance_ = params.color_importance_;
    this->spatial_importance_ = params.spatial_importance_;
    this->normal_importance_ = params.normal_importance_;

    this->use_extended_convexity_ = params.use_extended_convexity_;
    this->use_sanity_criterion_ = params.use_sanity_criterion_;
    this->concavity_tolerance_threshold_ = params.concavity_tolerance_threshold_;
    this->smoothness_threshold_ = params.smoothness_threshold_;

    this->initialized = true;
    this->detected_objects.resize(0);

}


void Segmentation::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{

   pcl::fromROSMsg(*input,*cloud);
   //this->input_msg_ .header.frame_id = *in
   this->input_msg_ = *input;

}

void Segmentation::detectObjectsOnTable(pcl::PointCloud<PointT>::Ptr cloud, double zmin, double zmax, bool filter_input_cloud)
{

  //Get the plane model, if Present
  pcl::SACSegmentation<PointT> seg;
  seg.setInputCloud(cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  seg.segment(*planeIndices, this->plane_coefficients);

  if (planeIndices->indices.size() == 0)
      ROS_INFO("Could not find a plane in the scene");


  else
  {
    // Copy the points of the plane to a new cloud.
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.filter(*table_plane_cloud);

    //Retrieve the convex Hull
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(table_plane_cloud);
    hull.setDimension(2);
    hull.reconstruct(this->convexHull);

    //Redundant Check
    if(hull.getDimension() == 2)
    {
      pcl::ExtractPolygonalPrismData<PointT> prism;
      prism.setInputCloud(cloud);
      prism.setInputPlanarHull(convexHull.makeShared());
      prism.setHeightLimits(zmin, zmax);
      pcl::PointIndices::Ptr obj_idx (new pcl::PointIndices());
      prism.segment(*obj_idx);
      extract.setIndices(obj_idx);
     if(filter_input_cloud)
       extract.filter(*cloud);

    }
  }
}

void Segmentation::segmentClusterIntoObjects(pcl::PointCloud<PointT>::Ptr clusters)
{

  if(this->seed_resolution_ < 0.013)
  {
    ROS_WARN("The seed resolution is really low. The segmentation could be fragmented");
  }
  pcl::SupervoxelClustering<PointT> super(this->voxel_resolution_, this->seed_resolution_);
  detected_objects.resize(0);
  if(clusters->points.size()  == 0)
  {
    ROS_ERROR("There is no object on the table");
  }
  super.setInputCloud(clusters);
  super.setColorImportance(this->color_importance_);
  super.setSpatialImportance(this->spatial_importance_);
  super.setNormalImportance(this->normal_importance_);

  super.extract(supervoxel_clusters);
  labeled_voxel_cloud = super.getLabeledVoxelCloud();
  pcl::PointCloud<PointT>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
  sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
  pcl::PointCloud<pcl::PointXYZL>::Ptr full_labeled_cloud = super.getLabeledCloud();

  super.getSupervoxelAdjacency(supervoxel_adjacency);
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr > refined_supervoxel_clusters;

  pcl::PointCloud<pcl::PointXYZL>::Ptr refined_labeled_voxel_cloud  = super.getLabeledVoxelCloud();
  pcl::PointCloud<pcl::PointNormal>::Ptr refined_sv_normal_cloud = super.makeSupervoxelNormalCloud(refined_supervoxel_clusters);
  pcl::PointCloud<pcl::PointXYZL>::Ptr refinded_full_labeled_cloud = super.getLabeledCloud();

  typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, uint32_t, float> VoxelAdjacencyList;
  VoxelAdjacencyList supervoxel_adjacency_list;
  super.getSupervoxelAdjacencyList(supervoxel_adjacency_list);

  //Segmentation
  uint k_factor = 0;
  if(this->use_extended_convexity_)
    k_factor = 1;

  pcl::LCCPSegmentation<PointT> lccp;
  lccp.setConcavityToleranceThreshold(this->concavity_tolerance_threshold_);
  lccp.setSanityCheck(this->use_sanity_criterion_);
  lccp.setSmoothnessCheck(true, this->voxel_resolution_, this->seed_resolution_, this->smoothness_threshold_);
  lccp.setKFactor(k_factor);
  lccp.setInputSupervoxels(this->supervoxel_clusters, this->supervoxel_adjacency);
  lccp.setMinSegmentSize(this->min_segment_size_);
  lccp.segment();

  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
  lccp_labeled_cloud = sv_labeled_cloud->makeShared();
  lccp.relabelCloud (*lccp_labeled_cloud);

  //Construct for each segmented object the point cloud associated to that object
  for(int i=0;i<lccp_labeled_cloud->points.size();++i)
  {
    uint32_t idx = lccp_labeled_cloud->points.at(i).label;
    if(idx >=detected_objects.size())
      detected_objects.resize(idx+1);

    PointT tmp_point;
    tmp_point = cloud->points.at(i);
    detected_objects[idx].object_cloud.points.push_back(tmp_point);
    detected_objects[idx].label = (int)idx;
  }

  int size = detected_objects.size();
  int i = 0;
  while(i<size)
  {
    if(detected_objects[i].object_cloud.size() < this->th_points_)
    {
      detected_objects.erase(detected_objects.begin() + i);
      size = detected_objects.size();
    }
    else
      i++;
  }

  std::cout<<"Detected objects: "<<detected_objects.size()<<std::endl;
  detectedObjectToCloud();


}

void Segmentation::detectedObjectToCloud()
{
  pcl::PointCloud<PointT> segmented_objects_cloud;
  for(int i=0;i<this->detected_objects.size();++i)
  {
    pcl::PointCloud<PointT> tmp;
    tmp = this->detected_objects[i].object_cloud;
    float r, g, b;
    r = static_cast<float> (rand())/static_cast<float> (RAND_MAX);
    g = static_cast<float> (rand())/static_cast<float> (RAND_MAX);
    b = static_cast<float> (rand())/static_cast<float> (RAND_MAX);
    for(int j=0;j<tmp.size(); ++j)
    {
      PointT temp_point;
      temp_point.x = tmp.points.at(j).x;
      temp_point.y = tmp.points.at(j).y;
      temp_point.z = tmp.points.at(j).z;
      temp_point.r = r*255;
      temp_point.g = g*255;
      temp_point.b = b*255;
      segmented_objects_cloud.points.push_back(temp_point);
    }
  }
  segmented_objects_cloud.width = segmented_objects_cloud.points.size();
  segmented_objects_cloud.height = 1;
  segmented_objects_cloud.is_dense = true;
  pcl::toROSMsg(segmented_objects_cloud, this->segmented_objects_msg);

  this->segmented_objects_msg.header.seq = 1;
  this->segmented_objects_msg.header.frame_id = this->input_msg_.header.frame_id;
  this->segmented_objects_msg.header.stamp = ros::Time::now();
}

void Segmentation::publish_clouds()
{
  pcl::toROSMsg(*table_plane_cloud, plane_cloud );
  pcl::toROSMsg(*cloud, object_cloud );
  pcl::toROSMsg(convexHull, convex_hull_cloud);
  cloud_pub_.publish(object_cloud);
  table_pub_.publish(plane_cloud);
  convex_hull_pub_.publish(convex_hull_cloud);
  segmented_objects_pub_.publish(segmented_objects_msg);

}

bool Segmentation::segment()
{

  this->initialized = true;
  ros::Rate rate(1);
  ROS_INFO("detecting............");
  while(ros::ok())
  {
   if(this->cloud->points.size() > 0)
   {
     detectObjectsOnTable(this->cloud, this->zmin_, this->zmax_, true);
     segmentClusterIntoObjects(this->cloud);

     publish_clouds();
   }
   ros::spinOnce();
   rate.sleep();
  }

}


void Segmentation::print_parameters()
{
    ROS_INFO ("ZMIN: %f", this->zmin_);
    ROS_INFO ("ZMAX: %f", this->zmax_);
}

void Segmentation::set_zmax(double zmax_in)
{
  this->zmax_ = zmax_in;
}

void Segmentation::set_zmin(double zmin_in)
{
  this->zmin_ = zmin_in;
}

double Segmentation::get_zmax()
{
  return this->zmax_;
}

double Segmentation::get_zmin()
{
  return this->zmin_;
}
