#include <sq_fitting/fitting.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <unsupported/Eigen/NonLinearOptimization>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>



SuperquadricFitting::SuperquadricFitting(const pcl::PointCloud<PointT>::Ptr& input_cloud) : pre_align_(true), pre_align_axis_(2)
{
  cloud_ = input_cloud;
  set_method_ = false;
}

void SuperquadricFitting::getMinParams(sq_fitting::sq &param)
{
  param = params_;
}

void SuperquadricFitting::getPreAlignedCloud(pcl::PointCloud<PointT>::Ptr& cloud)
{
  cloud = prealigned_cloud_;
}

void SuperquadricFitting::setPreAlign(bool pre_align, int pre_align_axis)
{
  pre_align_ = pre_align;
  pre_align_axis_ = pre_align_axis;
}

void SuperquadricFitting::preAlign(Eigen::Affine3f &transform, Eigen::Vector3f &variances)
{
  if(set_method_)
  {
    if(pose_est_method_=="iteration")
    {
        //centroid::my method
      double x, y, z;
      sq::getCenter(cloud_, x, y,z);
      Eigen::Affine3f transformation_centroid = Eigen::Affine3f::Identity();
      transformation_centroid.translation()<<-x, -y, -z;

      //Pose estimation
      //my method
      geometry_msgs::Pose pose;
      sq::getTransformPose(cloud_, pose);
      pose.position.x = 0;
      pose.position.y = 0;
      pose.position.z = 0;
      Eigen::Affine3d transform_in_eigen;
      tf::poseMsgToEigen(pose, transform_in_eigen);
      Eigen::Affine3d transform_in_eigen_inv = transform_in_eigen.inverse();
      transform =  transform_in_eigen_inv.cast<float>() * transformation_centroid ;
      Eigen::Vector3f eigenValues;
      eigenValues<<0.25,0.25,0.25;
      eigenValues /= static_cast<float>(cloud_->size());
      variances(0) = sqrt(eigenValues(0));
      variances(1) = sqrt(eigenValues(1));
      variances(2) = sqrt(eigenValues(2));
    }
  //////////////////////////////////////////////////////////////////////////////////
    if(pose_est_method_=="pca")
    {
      Eigen::Vector4f xyz_centroid;
        pcl::compute3DCentroid(*cloud_, xyz_centroid);
        Eigen::Affine3f transformation_centroid = Eigen::Affine3f::Identity();
        transformation_centroid.translation()<<-xyz_centroid(0), -xyz_centroid(1), -xyz_centroid(2);
        //Compute PCA
        pcl::PCA<PointT> pca;
        pca.setInputCloud(cloud_);
        Eigen::Vector3f eigenValues = pca.getEigenValues();
        Eigen::Matrix3f eigenVectors = pca.getEigenVectors();

        //std::cout<<"Eigen value from pca: "<<eigenValues<<std::endl;

        //Aligning first PCA axis with the prealign axis
        Eigen::Vector3f vec_aux = eigenVectors.col(0);
        eigenVectors.col(0) = eigenVectors.col(pre_align_axis_);
        eigenVectors.col(pre_align_axis_) = vec_aux;

        float aux_ev = eigenValues(0);
        eigenValues(0) = eigenValues(pre_align_axis_);
        eigenValues(pre_align_axis_) = aux_ev;

        Eigen::Matrix4f transformation_pca = Eigen::Matrix4f::Identity();

       for(int i =0 ;i<transformation_pca.cols()-1;++i)
        {
          for(int j = 0;j<transformation_pca.rows()-1;++j)
          {
            transformation_pca(j,i) = eigenVectors(i,j);
          }
        }

        Eigen::Affine3f transformation_pca_affine;
        transformation_pca_affine.matrix() = transformation_pca;
        transform = transformation_pca_affine * transformation_centroid;
        //transform = transformation_centroid*transformation_pca_affine ;

        //Eigen::Vector3f eigenValues;
        eigenValues /= static_cast<float>(cloud_->size());
        variances(0) = sqrt(eigenValues(0));
        variances(1) = sqrt(eigenValues(1));
        variances(2) = sqrt(eigenValues(2));
     }
  }
  //By moment of intertia estimation
  /*pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
  feature_extractor.setInputCloud (cloud_);
  feature_extractor.compute ();
  PointT  min_point_OBB;
  PointT  max_point_OBB;
  PointT position_OBB;
  PointT min_point_AABB;
  PointT max_point_AABB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  //Eigen::Affine3f transformation_centroid = Eigen::Affine3f::Identity();
  //transformation_centroid.translation()<<-position_OBB.x, -position_OBB.y, -position_OBB.z;
  //transformation_centroid.translation()<<-((min_point_AABB.x+max_point_AABB.x)/2),
      //-((min_point_AABB.y+max_point_AABB.y)/2),
      //-((min_point_AABB.z+max_point_AABB.z)/2);

  Eigen::Vector3f eigenValues;
  //eigenValues<<major_value, middle_value, minor_value;
  eigenValues<<0.25,0.25,0.25;
  //float aux_ev = eigenValues(0);
  //eigenValues(0) = eigenValues(pre_align_axis_);
  //eigenValues(pre_align_axis_) = aux_ev;

  /*Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
  transform_2.block<3,3>(0,0) = rotational_matrix_OBB.inverse();

  Eigen::EigenSolver<Eigen::Matrix4f > es(transform_2);
  std::cout<<"The eigenvalue"<<es.eigenvalues()<<std::endl;*/

 ////////////////////////////////////////////////////////////////////////////////////////

}

bool SuperquadricFitting::set_pose_est_method(const std::string method)
{
  if(method == "pca" || method== "iteration")
  {
    pose_est_method_ = method;
    set_method_ = true;
    return true;
  }
  else
    return false;
}

void SuperquadricFitting::fit_Param(sq_fitting::sq& param, double& final_error)
{
  Eigen::Affine3f transform_inv;
  Eigen::Vector3f variances;
  if(pre_align_)
  {
    preAlign(transform_inv, variances);
    prealigned_cloud_.reset(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud_, *prealigned_cloud_, transform_inv);
  }

  Eigen::Affine3d trans_new = (transform_inv.inverse()).cast<double>();
  double tx, ty, tz, ax, ay, az;
  sq::getParamFromPose(trans_new, tx, ty, tz, ax, ay, az);



  int n_unknowns = 11;
  Eigen::VectorXd xvec(n_unknowns);
  xvec[0] = variances(0) * 3.;
  xvec[1] = variances(1) * 3.;
  xvec[2] = variances(2) * 3.;
  xvec[3] = xvec[4] = 1.0;
  xvec[5] =  xvec[6] =  xvec[7] =  xvec[8] = xvec[9] =  xvec[10] =0.;
  /*xvec[5] = tx;
  xvec[6] = ty;
  xvec[7] = tz;
  xvec[8] = ax;
  xvec[9] = ay;
  xvec[10] = az;*/

  OptimizationFunctor functor(prealigned_cloud_->size(), this);
  Eigen::NumericalDiff<OptimizationFunctor> numericalDiffMyFunctor(functor);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm(numericalDiffMyFunctor);
  lm.minimize(xvec);

  param.a1 = xvec[0];
  param.a2 = xvec[1];
  param.a3 = xvec[2];
  double e1 = xvec[3];
  double e2 = xvec[4];
  sq::sq_clampParameters(e1, e2);
  param.e1 = e1;
  param.e2 = e2;

  Eigen::Affine3d transform_lm;
  //std::cout<<"Transformation frm LM: "<<xvec[5]<<" "<<xvec[6]<<" "<< xvec[7]<<" "<<xvec[8]<<" "<<xvec[9]<<" "<<xvec[10]<<std::endl;
  sq::create_transformation_matrix(xvec[5], xvec[6], xvec[7], xvec[8], xvec[9], xvec[10], transform_lm);

  Eigen::Affine3f transform = transform_inv.inverse();
  Eigen::Affine3d final_transform = transform.cast<double>();// * transform_lm ;
  //Eigen::Affine3d final_transform = transform.cast<double>() ;
  Eigen::Vector3d t = final_transform.translation();
  param.pose.position.x = t(0);
  param.pose.position.y = t(1);
  param.pose.position.z = t(2);
  Eigen::Matrix3d rot_matrix = final_transform.rotation();
  Eigen::Quaterniond q(rot_matrix);
  q.normalize();
  param.pose.orientation.x = q.x();
  param.pose.orientation.y = q.y();
  param.pose.orientation.z = q.z();
  param.pose.orientation.w = q.w();


  //Creating a new param with pose of transform_lm to calculate error
  sq_fitting::sq param_lm;
  param_lm.a1 = param.a1;
  param_lm.a2 = param.a2;
  param_lm.a3 = param.a3;
  param_lm.e1 = param.e1;
  param_lm.e2 = param.e2;
  Eigen::Vector3d t1 = transform_lm.translation();
  param_lm.pose.position.x = t1(0);
  param_lm.pose.position.y = t1(1);
  param_lm.pose.position.z = t1(2);
  Eigen::Matrix3d rot_matrix_lm = transform_lm.rotation();
  Eigen::Quaterniond q1(rot_matrix_lm);
  q1.normalize();
  param_lm.pose.orientation.x = q1.x();
  param_lm.pose.orientation.y = q1.y();
  param_lm.pose.orientation.z = q1.z();
  param_lm.pose.orientation.w = q1.w();
  final_error = sq::sq_error(cloud_, param_lm);
}

void SuperquadricFitting::fit()
{
  double min_fit_error = std::numeric_limits<double>::max();
  sq_fitting::sq min_param;
  for(int i=0;i<1;++i)
  {
    double error;
    setPreAlign(true, i);
    sq_fitting::sq param;
    fit_Param(param, error);
    //std::cout<<"Error is : "<<error<<std::endl;
    if(error<min_fit_error)
    {
      min_fit_error = error;
      min_param = param;
    }
  }
  params_ = min_param;
  min_error_ = min_fit_error; //std::cout<<"min error is: "<<min_error_<<std::endl;
}

void SuperquadricFitting::getMinError(double& error)
{
  error = min_error_;
}

int SuperquadricFitting::OptimizationFunctor::operator ()(const Eigen::VectorXd &xvec, Eigen::VectorXd &fvec) const
{
  pcl::PointCloud<PointT>::Ptr cloud_new(new pcl::PointCloud<PointT>);
  cloud_new = estimator_->prealigned_cloud_;
  double a = xvec[0], b = xvec[1],
              c = xvec[2], e1 = xvec[3],
              e2 =xvec[4];
  Eigen::Affine3d trans;
  sq::create_transformation_matrix(xvec[5], xvec[6], xvec[7], xvec[8], xvec[9], xvec[10], trans);
  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud_new, *cloud_transformed, trans);
  for(int i=0;i<values();++i)
  {
    Eigen::Matrix<double, 4, 1> xyz_tr (cloud_transformed->at(i).x, cloud_transformed->at(i).y, cloud_transformed->at(i).z, 1.);
    double op = Eigen::Matrix<double, 3, 1> (xyz_tr[0], xyz_tr[1], xyz_tr[2]).norm ();
    PointT p;
    p.x = xyz_tr[0];
    p.y = xyz_tr[1];
    p.z = xyz_tr[2];
    fvec[i] = op * sq::sq_function(xyz_tr[0], xyz_tr[1], xyz_tr[2], a,b,c,e1,e2) ;
  }
  return (0);
}

