#include <ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>

#include <string>
#include <iostream>
#include <iomanip>
#include <time.h>
#include <signal.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

void CloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromROSMsg(*msg, *cloud);

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;

  std::string serial = freenect2.getDefaultDeviceSerialNumber();
  std::cout << "SERIAL: " << serial << std::endl;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                    libfreenect2::Frame::Depth |
                                                    libfreenect2::Frame::Ir);
      libfreenect2::FrameMap frames;

      listener.waitForNewFrame(frames);
              libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
              libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
              libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2, rgbd3, coloreada, rgbdf;
    int xmin, xmax, ymin, ymax;
      float zmin, zmax;
    xmin = 675;
    xmax = xmin + 750;
    ymin = 150;
    ymax = ymin + 651;
    zmax = 1150.;
    zmin = 600.;

    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
            cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
            cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

            cv::rectangle(rgbmat, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(255,255,255));
                cv::imshow("rgb", rgbmat);
                cv::imshow("ir", irmat/4095.0f);
                cv::imshow("depth", depthmat/4095.0f);









  /*for(int i=0;i<10;++i)
    {
        std::cout<<cloud->at(i).x<<" "<<cloud->at(i).y<<" "<<cloud->at(i).z<<" "<<&(cloud->at(i)).rgb<<std::endl;
  }*/



}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_depth2rgb_cloud");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("kinect2/sd/points",1000, CloudCb);
  ros::spin();
  return 0;

  ROS_INFO("Hello world!");
}
