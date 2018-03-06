#ifndef FITTING_H
#define FITTING_H
#include <iostream>
#include <sq_fitting/sq.h>
#include <sq_fitting/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

//typedef
typedef pcl::PointXYZRGB PointT;

/**
 * @brief class for fitting superquadric parameters on point cloud data
 */
class SuperquadricFitting{
public:

  /**
   * @brief Copy Constructor
   * @param src
   */
  SuperquadricFitting(const SuperquadricFitting &src) {}

  /**
   * @brief Constructor initialize by point cloud data
   * @param input_cloud
   */
  SuperquadricFitting(const pcl::PointCloud<PointT>::Ptr& input_cloud);

  /**
   * @brief overloading operator
   * @param src
   * @return
   */
  SuperquadricFitting& operator = (const SuperquadricFitting &src){}
  //virtual ~SuperquadricFitting();

  /**
   * @brief setting pre align axis
   * @param pre_align
   * @param pre_align_axis
   */
  void setPreAlign(bool pre_align, int pre_align_axis =2);

  /**
   * @brief set preAlign and get transform and variances
   * @param transform
   * @param variances
   */
  void preAlign(Eigen::Affine3f& transform, Eigen::Vector3f& variances);

  /**
   * @brief fit params on current preAlign axis and return the minimum param and error
   * @param param
   * @param final_error
   */
  void fit_Param(sq_fitting::sq& param, double& final_error);

  /**
   * @brief fit param on different preAlign axis and set minimum param and minimum error
   */
  void fit();

  /**
   * @brief obtain the pre aligned cloud
   * @param cloud
   */
  void getPreAlignedCloud(pcl::PointCloud<PointT>::Ptr& cloud);

  /**
   * @brief obtain minimum params
   * @param param
   */
  void getMinParams(sq_fitting::sq& param);

  /**
   * @brief obtain minimum error
   * @param error
   */
  void getMinError(double& error);

  /**
   * @brief setting pose estimation method
   * @param method pca/iteration
   * @return
   */
  bool set_pose_est_method(const std::string method);



private:
  pcl::PointCloud<PointT>::Ptr cloud_;
  pcl::PointCloud<PointT>::Ptr prealigned_cloud_;
  sq_fitting::sq params_;
  bool pre_align_;
  int pre_align_axis_;
  double min_error_;
  std::string pose_est_method_;
  bool set_method_;

  ///typename _scalar
  template<typename _Scalar, int nX = Eigen::Dynamic, int nY = Eigen::Dynamic>

  /**
   * @brief The Functor struct
   */
  struct Functor
  {
    typedef _Scalar Scalar;
    enum
    {
      InputsAtCompileTime = nX,
      ValuesAtCompileTime = nY
    };
    typedef Eigen::Matrix<_Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<_Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    Functor() : m_data_points_(ValuesAtCompileTime) {}
    Functor(int m_data_points) : m_data_points_(m_data_points) {}
    virtual ~Functor(){}
    int values() const {return (m_data_points_);}
    int inputs() const {return 11;}
  protected:
    int m_data_points_;
  };

  /**
   * @brief The OptimizationFunctor struct
   */
  struct OptimizationFunctor : public Functor<double>
  {
    using Functor<double>::values;

    OptimizationFunctor (int m_data_points, SuperquadricFitting *estimator)
      :Functor<double> (m_data_points) , estimator_(estimator) {}

    inline OptimizationFunctor(const OptimizationFunctor *src)
      :Functor<double> (src->m_data_points_), estimator_()
    {
      *this = src;
    }


    virtual ~OptimizationFunctor() {}

    inline OptimizationFunctor& operator = (const OptimizationFunctor &src)
    {
      Functor<double>::operator=(src);
      estimator_ = src.estimator_;
      return (*this);
    }

    int operator () (const Eigen::VectorXd &xvec, Eigen::VectorXd &fvec) const;

    int df(const Eigen::VectorXd &xvec, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &fjac) const;

    SuperquadricFitting* estimator_;

  };
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif // FITTING_LM_H
