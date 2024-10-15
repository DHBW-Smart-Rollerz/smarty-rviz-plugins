#ifndef DYNAMIC_GROUND_IMAGE_HPP
#define DYNAMIC_GROUND_IMAGE_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreHardwarePixelBuffer.h> // Include the appropriate header

namespace dynamic_ground_image
{
class DynamicGroundImage : public rviz_common::Panel
{
  Q_OBJECT

public:
  DynamicGroundImage(QWidget * parent = nullptr);
  virtual ~DynamicGroundImage();

  virtual void onInitialize() override;

private Q_SLOTS:
  void updateImage();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  cv_bridge::CvImagePtr cv_ptr_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * scene_node_;
  Ogre::ManualObject * manual_object_;
};
}

#endif // DYNAMIC_GROUND_IMAGE_HPP
