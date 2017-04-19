#include <ros/ros.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;



  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("world", "/kinect2_ir_optical_frame",
                                    ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("world", "/kinect2_ir_optical_frame",
                               ros::Time(0), transform);
      std::cout<<"Transform: "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<transform.getOrigin().z()<<std::endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }



    rate.sleep();
  }
  return 0;
}
