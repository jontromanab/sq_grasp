#ifndef SQ_POSES_DISPLAY_H
#define SQ_POSES_DISPLAY_H

#include <geometry_msgs/PoseArray.h>
#include <rviz/message_filter_display.h>
#include <boost/ptr_container/ptr_vector.hpp>

namespace Ogre
{
  class ManualObject;
}

namespace  rviz
{
 class Axes;
 class FloatProperty;
}

namespace  sq_visualization
{

 class SqPoseArrayDisplay : public rviz::MessageFilterDisplay<geometry_msgs::PoseArray>
 {
   Q_OBJECT
 public:
   SqPoseArrayDisplay();
   virtual ~SqPoseArrayDisplay();

 protected:
   virtual void onInitialize();
   virtual void reset();
   virtual void processMessage(const geometry_msgs::PoseArray::ConstPtr& msg);

 private:
   bool setTransform(std_msgs::Header const & header);
   void updateAxes();
   void updateDisplay();
   rviz::Axes* makeAxes();

   struct OgrePose{
     Ogre::Vector3 position;
     Ogre::Quaternion orientation;
   };

   std::vector<OgrePose> poses_;
   std::vector<boost::shared_ptr<rviz::Axes> > axes_;

   Ogre::SceneNode* axes_node_;
   Ogre::ManualObject* manual_object_;
   rviz::FloatProperty* axes_length_property_;
   rviz::FloatProperty* axes_radius_property_;

 private Q_SLOTS:
   void updateAxesGeometry();

 };

} // end namespace sq_visualization
#endif // SQ_POSES_DISPLAY_H
