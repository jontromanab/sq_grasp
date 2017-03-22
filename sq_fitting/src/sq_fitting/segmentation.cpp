#include<sq_fitting/segmentation.h>

Segmentation::Segmentation(const CloudPtr& input_cloud, const Parameters& param)
  : table_plane_cloud_(new PointCloud), segmented_objects_cloud_(new PointCloud),
    objects_on_table_(new PointCloud)
{
  cloud_ = input_cloud;
  this->objects_.resize(0);

  this->zmin_ = param.zmin;
  this->zmax_ = param.zmax;
  this->th_points_ = param.th_points;
  this->voxel_resolution_ = param.voxel_resolution;
  this->seed_resolution_ = param.seed_resolution;
  this->color_importance_ = param.color_importance;
  this->spatial_importance_ = param.spatial_importance;
  this->normal_importance_ = param.normal_importance;
  this->use_extended_convexity_ = param.use_extended_convexity;
  this->use_sanity_criterion_ = param.use_sanity_criterion;
  this->concavity_tolerance_threshold_ = param.concavity_tolerance_threshold;
  this->smoothness_threshold_ = param.smoothness_threshold;
  this->initialized = true;
  this->detected_objects_.resize(0);
}

void Segmentation::detectObjectsOnTable(CloudPtr cloud, double zmin, double zmax, bool filter_input_cloud_)
{
  pcl::SACSegmentation<PointT> seg;
  seg.setInputCloud(cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  seg.segment(*planeIndices, this->plane_coefficients_);

  if(planeIndices->indices.size() == 0)
    ROS_INFO("Could not find a plane in the scene");
  else
  {
    //copy the points of the plane to a new cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.filter(*table_plane_cloud_);

    //Retrieve the convex hull
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(table_plane_cloud_);
    hull.setDimension(2);
    hull.reconstruct(this->convexHull);

    //Redundant check
    if(hull.getDimension() == 2)
    {
      pcl::ExtractPolygonalPrismData<PointT> prism;
      prism.setInputCloud(cloud);
      prism.setInputPlanarHull(convexHull.makeShared());
      prism.setHeightLimits(zmin, zmax);
      pcl::PointIndices::Ptr obj_idx(new pcl::PointIndices());
      prism.segment(*obj_idx);
      extract.setIndices(obj_idx);
      if(filter_input_cloud_)
      {
        extract.filter(*cloud);
        objects_on_table_ = cloud;
      }

    }
  }

}

void Segmentation::segmentClusterIntoObjects(CloudPtr clusters)
{
  if(this->seed_resolution_ < 0.013)
    {
      ROS_WARN("The seed resolution is really low. The segmentation could be fragmented");
    }
    pcl::SupervoxelClustering<PointT> super(this->voxel_resolution_, this->seed_resolution_);
    detected_objects_.resize(0);
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
      if(idx >=detected_objects_.size())
        detected_objects_.resize(idx+1);

      PointT tmp_point;
      tmp_point = cloud_->points.at(i);
      detected_objects_[idx].object_clouds.points.push_back(tmp_point);
      detected_objects_[idx].label = (int)idx;
    }

    int size = detected_objects_.size();
    int i = 0;
    while(i<size)
    {
      if(detected_objects_[i].object_clouds.size() < this->th_points_)
      {
        detected_objects_.erase(detected_objects_.begin() + i);
        size = detected_objects_.size();
      }
      else
        i++;
    }

    std::cout<<"Detected objects : "<<detected_objects_.size()<<std::endl;
    std::cout<<"=========================================================="<<std::endl;

}

void Segmentation::detectedObjectsToCloud()
{
  for(int i = 0;i<this->detected_objects_.size();++i)
  {
    CloudPtr object_cloud(new PointCloud);
    PointCloud obj_cloud = this->detected_objects_[i].object_clouds;
    object_cloud = obj_cloud.makeShared();
    float r, g, b;
    r = static_cast<float> (rand())/static_cast<float> (RAND_MAX);
    g = static_cast<float> (rand())/static_cast<float> (RAND_MAX);
    b = static_cast<float> (rand())/static_cast<float> (RAND_MAX);
    for(int j=0;j<object_cloud->points.size();++j)
    {
      object_cloud->points.at(j).r = r*255;
      object_cloud->points.at(j).g = g*255;
      object_cloud->points.at(j).b = b*255;
      segmented_objects_cloud_->points.push_back(object_cloud->points.at(j));
    }
    object_cloud->height = 1;
    object_cloud->width = object_cloud->points.size();
    object_cloud->is_dense = true;
    objects_.push_back(object_cloud);

  }
  segmented_objects_cloud_->height = 1;
  segmented_objects_cloud_->width = segmented_objects_cloud_->points.size();
  segmented_objects_cloud_->is_dense = true;
}

bool Segmentation::segment()
{
  if(this->initialized = true)
  {
    detectObjectsOnTable(this->cloud_, this->zmin_, this->zmax_,true);
    segmentClusterIntoObjects(this->cloud_);
    detectedObjectsToCloud();
  }
}

void Segmentation::getTablecloud(CloudPtr &table_cloud)
{
  table_cloud = table_plane_cloud_;
}

void Segmentation::getObjectsCloud(CloudPtr &object_cloud)
{
  object_cloud = segmented_objects_cloud_;
}

void Segmentation::getObjectsOnTable(CloudPtr &objects_on_table)
{
  objects_on_table = objects_on_table_;
}

void Segmentation::getObjects(std::vector<CloudPtr> &objects)
{
  objects = objects_;
}
