#include "wifi_viz/wifi_state_display.hpp"

#include <algorithm> // For std::max, std::min

// Ogre Headers
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <OgreRoot.h>
#include <OgreStringConverter.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreResourceGroupManager.h>

// Qt Headers
#include <QColor>
#include <QImage>
#include <QPainter>

// ROS Headers
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

// Custom Message Header
#include "wifi_viz/msg/min_max_curr.hpp"

namespace wifi_viz
{

using BaseDisplayClass = rviz_common::RosTopicDisplay<wifi_viz::msg::MinMaxCurr>;

WifiStateDisplay::WifiStateDisplay()
: BaseDisplayClass(),
  overlay_(nullptr),
  panel_(nullptr),
  needs_redraw_(false),
  current_value_(0.0f),
  min_value_(0.0f),
  max_value_(100.0f)
{
  static int instance_count = 0;
  instance_count++;

  overlay_name_ = "WifiVizOverlay" + Ogre::StringConverter::toString(instance_count);
  panel_name_ = "WifiVizPanel" + Ogre::StringConverter::toString(instance_count);
  material_name_ = "WifiVizMaterial" + Ogre::StringConverter::toString(instance_count);
  texture_name_ = "WifiVizTexture" + Ogre::StringConverter::toString(instance_count);

  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 200, "Width of the bar graph in pixels.",
    this, SLOT(updateProperties()), this);
  width_property_->setMin(10);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 20, "Height of the bar graph in pixels.",
    this, SLOT(updateProperties()), this);
  height_property_->setMin(5);

  left_property_ = new rviz_common::properties::IntProperty(
    "Left", 20, "Pixels from the left edge of the screen.",
    this, SLOT(updateProperties()), this);
  left_property_->setMin(0);

  top_property_ = new rviz_common::properties::IntProperty(
    "Top", 20, "Pixels from the top edge of the screen.",
    this, SLOT(updateProperties()), this);
  top_property_->setMin(0);

  frame_color_property_ = new rviz_common::properties::ColorProperty(
    "Frame Color", QColor(255, 255, 255), "Color of the bar's frame.",
    this, SLOT(updateProperties()), this);
}

WifiStateDisplay::~WifiStateDisplay()
{
}

void WifiStateDisplay::onInitialize()
{
  BaseDisplayClass::onInitialize();

  setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", "No topic selected");

  ensureOverlay();

  updateProperties();
  RVIZ_COMMON_LOG_INFO("WifiStateDisplay: Initialized.");
}

void WifiStateDisplay::ensureOverlay()
{
  if (overlay_) {
    return;
  }

  try {
    Ogre::OverlayManager & overlay_manager = Ogre::OverlayManager::getSingleton();

    overlay_ = overlay_manager.create(overlay_name_);
    overlay_->setZOrder(500);

    panel_ = static_cast<Ogre::PanelOverlayElement *>(
      overlay_manager.createOverlayElement("Panel", panel_name_));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    overlay_->add2D(panel_);

    material_ = Ogre::MaterialManager::getSingleton().getByName(material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    if (!material_) {
        material_ = Ogre::MaterialManager::getSingleton().create(
        material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    } else {
        material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }

    Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(false);
    pass->setCullingMode(Ogre::CULL_NONE);

    createTexture();

    panel_->setMaterialName(material_name_);

    overlay_->show();
    RVIZ_COMMON_LOG_INFO_STREAM("WifiStateDisplay: Created Ogre overlay '" << overlay_name_ << "'.");

  } catch (Ogre::Exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Error creating Ogre overlay: " << e.getDescription());
      overlay_ = nullptr;
      panel_ = nullptr;
  }
}

void WifiStateDisplay::onEnable()
{
  BaseDisplayClass::onEnable();
  // Show overlay when enabled
  if (overlay_) {
    overlay_->show();
  }
  needs_redraw_ = true; // Trigger redraw on enable
}

void WifiStateDisplay::onDisable()
{
  BaseDisplayClass::onDisable();
  // Hide overlay when disabled
  if (overlay_) {
    overlay_->hide();
  }
}

void WifiStateDisplay::update(float wall_dt, float ros_dt)
{
  // *** Crucial: Call the base class update method ***
  // This processes the message queue and calls processMessage() if new messages arrived.
  BaseDisplayClass::update(wall_dt, ros_dt);

  // Check if the overlay needs to be redrawn (flag set by processMessage or updateProperties)
  if (needs_redraw_) {
    // Check if overlay elements are valid before drawing
    if (overlay_ && panel_ && texture_ && !texture_image_.isNull()) {
        updateOverlayTexture();
        needs_redraw_ = false; // Reset the flag after redrawing
    }
  }
}

void WifiStateDisplay::reset()
{
  // Call base class reset
  BaseDisplayClass::reset();
  // Reset internal state and trigger redraw
  current_value_ = 0.0f;
  min_value_ = 0.0f;
  max_value_ = 100.0f; // Or some default
  needs_redraw_ = true;
}

void WifiStateDisplay::processMessage(wifi_viz::msg::MinMaxCurr::ConstSharedPtr msg)
{
  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");

  current_value_ = msg->current;
  min_value_ = msg->min;
  max_value_ = msg->max;

  needs_redraw_ = true;

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "WifiStateDisplay: Received current=" << current_value_ <<
      ", min=" << min_value_ << ", max=" << max_value_);
}

void WifiStateDisplay::updateProperties()
{
  ensureOverlay();

  if (!overlay_ || !panel_) {
    RVIZ_COMMON_LOG_WARNING("WifiStateDisplay: Overlay not ready, cannot update properties.");
    return;
  }

  int width = width_property_->getInt();
  int height = height_property_->getInt();
  int left = left_property_->getInt();
  int top = top_property_->getInt();

  try {
    panel_->setDimensions(static_cast<Ogre::Real>(width), static_cast<Ogre::Real>(height));
    panel_->setPosition(static_cast<Ogre::Real>(left), static_cast<Ogre::Real>(top));
  } catch (Ogre::Exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Error setting panel dimensions/position: " << e.getDescription());
      return;
  }

  if (!texture_ || texture_->getWidth() != (unsigned int)width || texture_->getHeight() != (unsigned int)height) {
    createTexture();
  } else {
    needs_redraw_ = true;
  }

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "WifiStateDisplay: Updated properties: pos=(" << left << "," << top <<
      "), size=(" << width << "," << height << ")");
}

void WifiStateDisplay::createTexture()
{
  ensureOverlay();

  if (!material_) {
      RVIZ_COMMON_LOG_ERROR("WifiStateDisplay: Material is null, cannot create texture.");
      return;
  }

  int width = width_property_->getInt();
  int height = height_property_->getInt();

  if (width <= 0 || height <= 0) {
    RVIZ_COMMON_LOG_WARNING("WifiStateDisplay: Invalid dimensions, cannot create texture.");
    return;
  }

  try {
    if (texture_ && Ogre::TextureManager::getSingleton().resourceExists(texture_name_)) {
        Ogre::TextureManager::getSingleton().remove(texture_name_);
    }
    texture_.reset();

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width, height, 0, Ogre::PF_A8R8G8B8,
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
    if (pass->getNumTextureUnitStates() > 0) {
      pass->getTextureUnitState(0)->setTexture(texture_);
    } else {
      pass->createTextureUnitState(texture_name_);
    }
    pass->getTextureUnitState(0)->setTextureFiltering(Ogre::TFO_NONE);

    texture_image_ = QImage(width, height, QImage::Format_ARGB32);
    needs_redraw_ = true;

    RVIZ_COMMON_LOG_DEBUG_STREAM("WifiStateDisplay: Created/Recreated texture " << texture_name_);

  } catch (Ogre::Exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Error creating/recreating Ogre texture: " << e.getDescription());
      texture_.reset();
  }
}

void WifiStateDisplay::updateOverlayTexture()
{
  if (!texture_ || texture_image_.isNull()) {
    RVIZ_COMMON_LOG_WARNING("WifiStateDisplay: Texture or QImage not ready, skipping redraw.");
    return;
  }

  texture_image_.fill(Qt::transparent);

  QPainter painter(&texture_image_);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QColor frame_color = frame_color_property_->getColor();
  int width = texture_image_.width();
  int height = texture_image_.height();

  float value_range = max_value_ - min_value_;
  float clamped_value = std::max(min_value_, std::min(max_value_, current_value_));
  float percentage = (value_range > 1e-6) ? (clamped_value - min_value_) / value_range : 0.0f;

  int frame_thickness = 2;
  int available_width = width - 2 * frame_thickness;
  int bar_width = std::max(0, static_cast<int>(available_width * percentage));

  painter.setPen(QPen(frame_color, frame_thickness));
  painter.setBrush(Qt::transparent);
  painter.drawRect(frame_thickness / 2, frame_thickness / 2,
                   width - frame_thickness, height - frame_thickness);

  QColor bar_color;
  if (percentage < 0.5f) {
    bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0);
  } else {
    bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0);
  }

  if (bar_width > 0) {
      painter.setPen(Qt::NoPen);
      painter.setBrush(bar_color);
      painter.drawRect(frame_thickness, frame_thickness,
                       bar_width, height - 2 * frame_thickness);
  }

  painter.end();

  try {
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox & pixel_box = pixel_buffer->getCurrentLock();
    uint8_t * pDest = static_cast<uint8_t *>(pixel_box.data);
    size_t ogre_bytes_per_line = pixel_box.rowPitch * Ogre::PixelUtil::getNumElemBytes(pixel_box.format);

    if (ogre_bytes_per_line == static_cast<size_t>(texture_image_.bytesPerLine())) {
      memcpy(pDest, texture_image_.constBits(), texture_image_.sizeInBytes());
    } else {
      for (int y = 0; y < height; ++y) {
        memcpy(
          pDest + y * ogre_bytes_per_line,
          texture_image_.constScanLine(y),
          texture_image_.bytesPerLine());
      }
    }
    pixel_buffer->unlock();
    RVIZ_COMMON_LOG_DEBUG("WifiStateDisplay: Overlay texture updated.");
  } catch (Ogre::Exception & e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error uploading texture data: " << e.getDescription());
  }
}

} // namespace wifi_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wifi_viz::WifiStateDisplay, rviz_common::Display)
