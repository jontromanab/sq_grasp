#include<sq_fitting/utils.h>
//#include <ceres/jet.h>







void sq_clampParameters(double& e1_clamped, double& e2_clamped)
{
  if(e1_clamped < 0.1)
    e1_clamped = 0.1;
  else if(e1_clamped > 1.9)
    e1_clamped = 1.9;
  if(e2_clamped < 0.1)
    e2_clamped = 0.1;
  else if(e2_clamped > 1.9)
    e2_clamped = 1.9;
}


double sq_function(const PointT& point, const sq_fitting::sq &param)
{
  double e1_clamped = param.e1;
  double e2_clamped = param.e2;
  sq_clampParameters(e1_clamped, e2_clamped);
  double t1 = pow(std::abs(point.x / param.a1), 2.0/e2_clamped);
  double t2 = pow(std::abs(point.y / param.a2), 2.0/e2_clamped);
  double t3 = pow(std::abs(point.z / param.a3), 2.0/e1_clamped);
  double f = pow(std::abs(t1+t2), e2_clamped/e1_clamped) + t3;
  return (pow(f,e1_clamped) - 1.0 );
}

double sq_function(const double &x, const double &y, const double &z, const double &a, const double &b, const double &c, const double &e1,
                   const double &e2)
{
  double e1_clamped = e1;
  double e2_clamped = e2;
  sq_clampParameters(e1_clamped, e2_clamped);

  //std::cout<<"e1: "<<e1_clamped<<" e2: "<<e2_clamped<<std::endl;
  double t1 = pow(std::abs(x / a), 2.0/e2_clamped);
  double t2 = pow(std::abs(y / b), 2.0/e2_clamped);
  double t3 = pow(std::abs(z / c), 2.0/e1_clamped);
  double f = pow(std::abs(t1+t2), e2_clamped/e1_clamped) + t3;
  double value = (pow(f, e1_clamped/2.0) - 1.0) * pow(a * b * c, 0.25);
  return (value);

}


double sq_function_scale_weighting(const PointT& point, const sq_fitting::sq &param)
{
  double e1_clamped = param.e1;
  double e2_clamped = param.e2;
  sq_clampParameters(e1_clamped, e2_clamped);
  double t1 = pow(std::abs(point.x / param.a1), 2.0/e2_clamped);
  double t2 = pow(std::abs(point.y /param.a2), 2.0/e2_clamped);
  double t3 = pow(std::abs(point.z /param.a3), 2.0/e1_clamped);
  double f = pow(std::abs(t1+t2), e2_clamped/e1_clamped) + t3;
  return (pow(f, e1_clamped/2.0) - 1.0) * pow(param.a1 * param.a2 * param.a3, 0.25);
}

double sq_error(const pcl::PointCloud<PointT>::Ptr cloud, const sq_fitting::sq &param)
{
  Eigen::Affine3f transform;
  sq_create_transform(param.pose, transform);
  pcl::PointCloud<PointT>::Ptr new_cloud(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud, *new_cloud, transform);
  double error = 0.0;
  for(size_t i = 0;i<new_cloud->size();++i)
  {
    double op = sq_normPoint(new_cloud->at(i));
    double val = op * sq_function_scale_weighting(new_cloud->at(i), param);
    error += val * val;
  }
  error /= new_cloud->size();
  return error;
}

void sq_create_transform(const geometry_msgs::Pose& pose, Eigen::Affine3f& transform)
{
  transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.translation()<<pose.position.x, pose.position.y, pose.position.z;
  transform.rotate(q);
}

double sq_normPoint(const PointT &point)
{
  double value = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  return value;
}


void euler2Quaternion(const double roll, const double pitch, const double yaw, Eigen::Quaterniond &q)
{
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  q = yawAngle * pitchAngle * rollAngle;
}

void create_transformation_matrix(const double tx, const double ty, const double tz, const double ax, const double ay, const double az, Eigen::Affine3d &trns_mat)
{
  trns_mat = Eigen::Affine3d::Identity();
  Eigen::Affine3d rot_matrix = Eigen::Affine3d::Identity();
  create_rotation_matrix(ax, ay, az, rot_matrix);
  Eigen::Affine3d translation_matrix(Eigen::Translation3d(Eigen::Vector3d(tx, ty, tz)));
  trns_mat = translation_matrix*rot_matrix;// * translation_matrix;
}

void create_rotation_matrix(const double ax, const double ay, const double az, Eigen::Affine3d &rot_matrix)
{
  Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1,0,0)));
  Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0,1,0)));
  Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0,0,1)));
  rot_matrix =  rx * rz * ry;
}

void getParamFromPose(const geometry_msgs::Pose &pose, double &tx, double &ty, double &tz, double &ax, double &ay, double &az)
{
  Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
  sq_create_transform(pose, transformation);
  Eigen::Affine3d transformation_d = transformation.cast<double>();
  Eigen::Vector3d t = transformation_d.translation();
  Eigen::Matrix3d rot_matrix =transformation_d.rotation();
  tx = t(0);
  ty = t(1);
  tz = t(2);
  Eigen::Vector3d rot_angle = rot_matrix.eulerAngles(0,1,2);
  ax = rot_angle[0] ;
  ay = rot_angle[1] ;
  az = rot_angle[2] ;
}


void getParamFromPose(const Eigen::Affine3d &trans, double &tx, double &ty, double &tz, double &ax, double &ay, double &az)
{
  Eigen::Vector3d t = trans.translation();
  Eigen::Matrix3d rot_matrix =trans.rotation();
  tx = t(0);
  ty = t(1);
  tz = t(2);
  Eigen::Vector3d rot_angle = rot_matrix.eulerAngles(0,1,2);
  ax = rot_angle[0] ;
  ay = rot_angle[1] ;
  az = rot_angle[2] ;

}

void Quaternion2Euler(const geometry_msgs::Pose &pose, double &ax, double &ay, double &az)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0,1,2);
  euler[0] = ax;
  euler[1] = ay;
  euler[2] = az;

}




void getCenter(pcl::PointCloud<PointT>::Ptr& cloud_in, double& x, double& y, double& z)
{
  double val_x = cloud_in->points.at(0).x;
  double val_y = cloud_in->points.at(0).y;
  double val_z = cloud_in->points.at(0).z;
  double x_max = val_x, x_min = val_x, y_max = val_y, y_min = val_y, z_max = val_z, z_min = val_z;
  for(int i=0;i<cloud_in->points.size();++i)
  {
    if (cloud_in->points.at(i).x>=x_max)
      x_max = cloud_in->points.at(i).x;
    if (cloud_in->points.at(i).x<=x_min)
      x_min = cloud_in->points.at(i).x;
    if (cloud_in->points.at(i).y>=y_max)
      y_max = cloud_in->points.at(i).y;
    if (cloud_in->points.at(i).y<=y_min)
      y_min = cloud_in->points.at(i).y;
    if (cloud_in->points.at(i).z>=z_max)
      z_max = cloud_in->points.at(i).z;
    if (cloud_in->points.at(i).z<=z_min)
      z_min = cloud_in->points.at(i).z;
  }
  x = (x_max+x_min)/2;
  y = (y_max+y_min)/2;
  z = (z_max+z_min)/2;

}


void getTransformPose(pcl::PointCloud<PointT>::Ptr& cloud_in, geometry_msgs::Pose &pose)
{

  // Compute z height as maximum distance from planes
  double val_z = cloud_in->points.at(0).z;
  double  z_max = val_z, z_min = val_z;
  for(int i=0;i<cloud_in->points.size();++i)
  {
    if (cloud_in->points.at(i).z>=z_max)
      z_max = cloud_in->points.at(i).z;
    if (cloud_in->points.at(i).z<=z_min)
      z_min = cloud_in->points.at(i).z;
  }

  double height = z_max -z_min;
  double min_volume = std::numeric_limits<double>::max();
  Eigen::Matrix3f transformation;
  // Find the convex hull
  pcl::PointCloud<pcl::PointXYZRGB> hull;
  pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
  convex_hull.setInputCloud(cloud_in);
  convex_hull.setDimension(2);
  convex_hull.reconstruct(hull);

  // Try fitting a rectangle
  for (size_t i = 0; i < hull.size() - 1; ++i) {
  // For each pair of hull points, determine the angle
    double rise = hull[i + 1].y - hull[i].y;
    double run = hull[i + 1].x - hull[i].x;
    // and normalize..
    {
      double l = sqrt((rise * rise) + (run * run));
      rise = rise / l;
      run = run / l;
     }

    // Build rotation matrix from change of basis
    Eigen::Matrix3f rotation;
    rotation(0, 0) = run;
    rotation(0, 1) = rise;
    rotation(0, 2) = 0.0;
    rotation(1, 0) = -rise;
    rotation(1, 1) = run;
    rotation(1, 2) = 0.0;
    rotation(2, 0) = 0.0;
    rotation(2, 1) = 0.0;
    rotation(2, 2) = 1.0;
    Eigen::Matrix3f inv_rotation = rotation.inverse();

    // Project hull to new coordinate system
    pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
    for (size_t j = 0; j < hull.size(); ++j)
    {
        pcl::PointXYZRGB p;
        p.getVector3fMap() = rotation * hull[j].getVector3fMap();
        projected_cloud.push_back(p);
    }

    // Compute min/max
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    for (size_t j = 0; j < projected_cloud.size(); ++j)
    {
       if (projected_cloud[j].x < x_min) x_min = projected_cloud[j].x;
       if (projected_cloud[j].x > x_max) x_max = projected_cloud[j].x;

       if (projected_cloud[j].y < y_min) y_min = projected_cloud[j].y;
       if (projected_cloud[j].y > y_max) y_max = projected_cloud[j].y;
    }


    // Is this the best estimate?
    double area = (x_max - x_min) * (y_max - y_min);
    if (area * height < min_volume)
    {
      //transformation = inv_plane_rotation * inv_rotation;
      transformation = inv_rotation;
      //dimensions->x = (x_max - x_min);
     // dimensions->y = (y_max - y_min);
      //dimensions->z = height;

      Eigen::Vector3f pose3f((x_max + x_min) / 2.0, (y_max + y_min) / 2.0, projected_cloud[0].z + height / 2.0);
      pose3f = transformation * pose3f;
      pose.position.x = pose3f(0);
      pose.position.y = pose3f(1);
      pose.position.z = pose3f(2);
      Eigen::Quaternionf q(transformation);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      min_volume = area * height;
    }
   }
}



void getCompletePose(pcl::PointCloud<PointT>::Ptr& cloud_in, geometry_msgs::Pose &pose)
{

  // Compute y height as maximum distance from planes
  double val_x = cloud_in->points.at(0).x;
  double  x_max = val_x, x_min = val_x;
  for(int i=0;i<cloud_in->points.size();++i)
  {
    if (cloud_in->points.at(i).x>=x_max)
      x_max = cloud_in->points.at(i).x;
    if (cloud_in->points.at(i).x<=x_min)
      x_min = cloud_in->points.at(i).x;
  }

  double height = x_max -x_min;
  double min_volume = std::numeric_limits<double>::max();
  Eigen::Matrix3f transformation;
  // Find the convex hull
  pcl::PointCloud<pcl::PointXYZRGB> hull;
  pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
  convex_hull.setInputCloud(cloud_in);
  convex_hull.setDimension(2);
  convex_hull.reconstruct(hull);

  // Try fitting a rectangle
  for (size_t i = 0; i < hull.size() - 1; ++i) {
  // For each pair of hull points, determine the angle
    double rise = hull[i + 1].y - hull[i].y;
    double run = hull[i + 1].z - hull[i].z;
    // and normalize..
    {
      double l = sqrt((rise * rise) + (run * run));
      rise = rise / l;
      run = run / l;
     }

    // Build rotation matrix from change of basis
    Eigen::Matrix3f rotation;
    rotation(0, 0) = run;
    rotation(0, 1) = rise;
    rotation(0, 2) = 0.0;
    rotation(1, 0) = -rise;
    rotation(1, 1) = run;
    rotation(1, 2) = 0.0;
    rotation(2, 0) = 0.0;
    rotation(2, 1) = 0.0;
    rotation(2, 2) = 1.0;
    Eigen::Matrix3f inv_rotation = rotation.inverse();

    // Project hull to new coordinate system
    pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
    for (size_t j = 0; j < hull.size(); ++j)
    {
        pcl::PointXYZRGB p;
        p.getVector3fMap() = rotation * hull[j].getVector3fMap();
        projected_cloud.push_back(p);
    }

    // Compute min/max
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::min();
    for (size_t j = 0; j < projected_cloud.size(); ++j)
    {
       if (projected_cloud[j].y < y_min) y_min = projected_cloud[j].y;
       if (projected_cloud[j].y > y_max) y_max = projected_cloud[j].y;

       if (projected_cloud[j].z < z_min) z_min = projected_cloud[j].z;
       if (projected_cloud[j].z > z_max) z_max = projected_cloud[j].z;
    }


    // Is this the best estimate?
    double area = (y_max - y_min) * (z_max - z_min);
    if (area * height < min_volume)
    {
      //transformation = inv_plane_rotation * inv_rotation;
      transformation = inv_rotation;
      //dimensions->x = (x_max - x_min);
     // dimensions->y = (y_max - y_min);
      //dimensions->z = height;

      Eigen::Vector3f pose3f(projected_cloud[0].x + height / 2.0,(y_max + y_min) / 2.0, (z_max + z_min) / 2.0);
      pose3f = transformation * pose3f;
      pose.position.x = pose3f(0);
      pose.position.y = pose3f(1);
      pose.position.z = pose3f(2);
      Eigen::Quaternionf q(transformation);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      min_volume = area * height;
    }
   }
}

