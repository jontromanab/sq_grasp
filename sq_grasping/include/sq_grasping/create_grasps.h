#ifndef CREATE_GRASPS_H
#define CREATE_GRASPS_H

#include<sq_fitting/sqArray.h>
#include<grasp_execution/graspArr.h>
#include<geometry_msgs/Pose.h>
#include<moveit/move_group_interface/move_group.h>

class CreateGrasps
{
public:
  CreateGrasps(sq_fitting::sqArray sqArr, const std::string group, const std::string ee_name);
  ~CreateGrasps();
  void sample_initial_grasps();
  void getGrasps(grasp_execution::graspArr &grasps);
private:
  sq_fitting::sqArray sqArr_;
  grasp_execution::graspArr init_grasps_;
  std::string frame_id_;
  std::string ee_name_;

  void createGraspPoses(const sq_fitting::sq& sq,
                        std::vector<geometry_msgs::Pose>& poses);

  moveit::planning_interface::MoveGroup group_;



};

#endif // CREATE_GRASPS_H
