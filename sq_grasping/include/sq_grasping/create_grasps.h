#ifndef CREATE_GRASPS_H
#define CREATE_GRASPS_H

#include<sq_fitting/sqArray.h>
#include<grasp_execution/graspArr.h>

class CreateGrasps
{
public:
  CreateGrasps(sq_fitting::sqArray sqArr);
  ~CreateGrasps();
private:
  sq_fitting::sqArray sqArr_;

};

#endif // CREATE_GRASPS_H
