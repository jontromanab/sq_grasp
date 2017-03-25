#include "sq_poses_display.h"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/validate_floats.h>

namespace sq_visualization
{
  SqPoseArrayDisplay::SqPoseArrayDisplay() : manual_object_(NULL)
  {
    axes_length_property_ = new rviz::FloatProperty("Axes Length", 0.3, "Length of each axis", this, SLOT(updateAxesGeometry()));
    axes_radius_property_ = new rviz::FloatProperty("Axes radius", 0.01, "Radius of each axis", this, SLOT(updateAxesGeometry()));
    axes_.resize(0);
  }

  SqPoseArrayDisplay::~SqPoseArrayDisplay()
  {
    if(initialized())
    {
      scene_manager_->destroyManualObject(manual_object_);
    }
  }

  void SqPoseArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    manual_object_ = scene_manager_->createManualObject();
    manual_object_->setDynamic(true);
    scene_node_->attachObject(manual_object_);
    axes_node_ = scene_node_->createChildSceneNode();
  }

  bool validateFloats(const geometry_msgs::PoseArray& msg)
  {
    return rviz::validateFloats(msg.poses);
  }

  void SqPoseArrayDisplay::processMessage(const geometry_msgs::PoseArray::ConstPtr &msg)
  {
    if(!validateFloats(*msg))
    {
      setStatus(rviz::StatusProperty::Error, "Topic", "Message contains invalid floating point values(nans of infs");
      return ;
    }

    if(!setTransform(msg->header))
    {
      setStatus(rviz::StatusProperty::Error, "Topic", "Failed to loop up transform");
      return;
    }
    poses_.resize(0);
    axes_.resize(0);
    for(std::size_t i=0;i<msg->poses.size();++i)
    {
      Ogre::Vector3 position_(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
      Ogre::Quaternion orientation_(msg->poses[i].orientation.w, msg->poses[i].orientation.x, msg->poses[i].orientation.y, msg->poses[i].orientation.z);
      boost::shared_ptr<rviz::Axes> axe_single;
      axe_single.reset(new rviz::Axes(scene_manager_, axes_node_, axes_length_property_->getFloat(),
                                             axes_radius_property_->getFloat()));
      axe_single->setPosition(position_);
      axe_single->setOrientation(orientation_);
      axes_.push_back(axe_single);
    }
    //updateAxes();
    context_->queueRender();
  }

  bool SqPoseArrayDisplay::setTransform(const std_msgs::Header &header)
  {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(header, position, orientation))
    {
      ROS_ERROR("Error transforming pose '%s' from frame'%s' to frame '%s' ",
                qPrintable(getName()), header.frame_id.c_str(),
                qPrintable(fixed_frame_));
      return false;
    }
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    return true;
  }


  void SqPoseArrayDisplay::updateAxesGeometry()
  {
    for(std::size_t i=0;i<poses_.size();++i)
    {
      axes_[i]->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
    }
    context_->queueRender();
  }

  void SqPoseArrayDisplay::reset()
  {
    MFDClass::reset();
    if(manual_object_)
    {
      manual_object_->clear();
    }
    axes_.clear();
  }


}// namespace sq_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( sq_visualization::SqPoseArrayDisplay, rviz::Display )
