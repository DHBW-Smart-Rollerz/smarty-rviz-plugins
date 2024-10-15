#include "dynamic_ground_image/dynamic_ground_image.hpp"
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreTextureManager.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreHardwarePixelBuffer.h> // Include the appropriate header

namespace dynamic_ground_image
{
DynamicGroundImage::DynamicGroundImage(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * layout = new QVBoxLayout;
  setLayout(layout);
}

DynamicGroundImage::~DynamicGroundImage()
{
    // Cleanup if necessary
}

void DynamicGroundImage::onInitialize()
{
  rviz_common::Panel::onInitialize();
  scene_manager_ = getDisplayContext()->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // Create a manual object for the ground plane
  manual_object_ = scene_manager_->createManualObject();
  manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  manual_object_->position(-5.0, 0.0, -5.0);
  manual_object_->textureCoord(0.0, 0.0);
  manual_object_->position(5.0, 0.0, -5.0);
  manual_object_->textureCoord(1.0, 0.0);
  manual_object_->position(5.0, 0.0, 5.0);
  manual_object_->textureCoord(1.0, 1.0);
  manual_object_->position(-5.0, 0.0, 5.0);
  manual_object_->textureCoord(0.0, 1.0);
  manual_object_->quad(0, 1, 2, 3);
  manual_object_->end();

  scene_node_->attachObject(manual_object_);

    // Create a texture and material for the ground plane
  texture_ = Ogre::TextureManager::getSingleton().createManual(
      "GroundImageTexture",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      512, 512,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  material_ = Ogre::MaterialManager::getSingleton().create(
      "GroundImageMaterial",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->getTechnique(0)->getPass(0)->createTextureUnitState("GroundImageTexture");
  material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  manual_object_->setMaterialName(0, "GroundImageMaterial");

    // Subscribe to the image topic
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&DynamicGroundImage::imageCallback, this, std::placeholders::_1));
}

void DynamicGroundImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
    updateImage();
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
        "cv_bridge exception: %s", e.what());
  }
}

void DynamicGroundImage::updateImage()
{
  if (!cv_ptr_) {
    return;
  }

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox & pixel_box = pixel_buffer->getCurrentLock();
  uint8_t * pDest = static_cast<uint8_t *>(pixel_box.data);

  cv::Mat image(cv_ptr_->image.rows, cv_ptr_->image.cols, CV_8UC3, pDest);
  cv::resize(cv_ptr_->image, image, cv::Size(512, 512));
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

  pixel_buffer->unlock();
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(dynamic_ground_image::DynamicGroundImage, rviz_common::Panel)
