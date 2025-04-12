#include "wifi_viz/battery_state_display.hpp"

#include <algorithm> // For std::max, std::min

// Ogre Headers - Use component subdirectories
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <Overlay/OgreOverlay.h> // Changed path
#include <Overlay/OgreOverlayManager.h> // Changed path
#include <Overlay/OgrePanelOverlayElement.h> // Changed path
#include <OgreRoot.h> // Usually top-level
#include <OgreStringConverter.h> // Usually top-level
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreResourceGroupManager.h>

#include <QColor>
#include <QImage>
#include <QPainter>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>

namespace wifi_viz
{

using BaseDisplayClass = rviz_common::RosTopicDisplay<sensor_msgs::msg::BatteryState>;

BatteryStateDisplay::BatteryStateDisplay()
: BaseDisplayClass(),
  overlay_(nullptr),
  panel_(nullptr),
  needs_redraw_(true),
  current_voltage_(0.0f)
{
  // Generate unique names for Ogre resources
  static int instance_count = 0;
  instance_count++;
  overlay_name_ = "BatteryOverlay" + Ogre::StringConverter::toString(instance_count);
  panel_name_ = "BatteryPanel" + Ogre::StringConverter::toString(instance_count);
  material_name_ = "BatteryMaterial" + Ogre::StringConverter::toString(instance_count);
  texture_name_ = "BatteryTexture" + Ogre::StringConverter::toString(instance_count);

  // --- Initialize Properties ---
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 200, "Width of the battery bar in pixels.",
    this, SLOT(updateProperties()), this);
  width_property_->setMin(10);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 20, "Height of the battery bar in pixels.",
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

BatteryStateDisplay::~BatteryStateDisplay()
{
  if (overlay_) {
    Ogre::OverlayManager::getSingleton().destroy(overlay_);
    // Panel is destroyed by overlay manager
  }
  if (!material_.isNull()) {
    Ogre::MaterialManager::getSingleton().remove(material_name_);
  }
  if (!texture_.isNull()) {
    Ogre::TextureManager::getSingleton().remove(texture_name_);
  }
}

void BatteryStateDisplay::onInitialize()
{
  BaseDisplayClass::onInitialize(); // Initialize base class

  // Set an initial status message after base class init
  setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", "No topic selected");

  // Ensure Ogre overlay elements are created
  ensureOverlay();

  // Set initial properties and trigger first draw
  updateProperties();
  RVIZ_COMMON_LOG_INFO("BatteryStateDisplay: Initialized.");
}

void BatteryStateDisplay::ensureOverlay()
{
  if (overlay_) {
    return; // Already created
  }

  Ogre::OverlayManager & overlay_manager = Ogre::OverlayManager::getSingleton();
  overlay_ = overlay_manager.create(overlay_name_);

  // Create the panel overlay element (container for the texture)
  panel_ = static_cast<Ogre::PanelOverlayElement *>(
    overlay_manager.createOverlayElement("Panel", panel_name_));
  panel_->setMetricsMode(Ogre::GMM_PIXELS); // Use pixel coordinates
  overlay_->add2D(panel_);                 // Add panel to overlay

  // Create the material for the panel
  material_ = Ogre::MaterialManager::getSingleton().getByName(material_name_);
  if (material_.isNull()) {
    material_ = Ogre::MaterialManager::getSingleton().create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }
  material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  material_->setCullingMode(Ogre::CULL_NONE); // Show back faces if needed

  panel_->setMaterialName(material_name_);

  // Create texture (will be done in updateProperties/createTexture)
  createTexture();

  overlay_->show(); // Make overlay visible
}

void BatteryStateDisplay::onEnable()
{
  BaseDisplayClass::onEnable();
  if (overlay_) {
    overlay_->show();
  }
}

void BatteryStateDisplay::onDisable()
{
  BaseDisplayClass::onDisable();
  if (overlay_) {
    overlay_->hide();
  }
}

void BatteryStateDisplay::update(float wall_dt, float ros_dt)
{
  ensureOverlay();

  if (needs_redraw_ && overlay_ && panel_) {
    updateOverlayTexture();
    needs_redraw_ = false;
  }
}

void BatteryStateDisplay::reset()
{
  BaseDisplayClass::reset();
  current_voltage_ = 0.0f;
  needs_redraw_ = true;
  updateProperties(); // Re-apply properties
}

void BatteryStateDisplay::processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  // Update status when messages are received
  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");

  current_voltage_ = msg->voltage;
  needs_redraw_ = true; // Mark for redraw in the next update() call

  RVIZ_COMMON_LOG_DEBUG_STREAM("BatteryStateDisplay: Received voltage: " << current_voltage_);
}

void BatteryStateDisplay::updateProperties()
{
  ensureOverlay(); // Ensure overlay exists

  int width = width_property_->getInt();
  int height = height_property_->getInt();
  int left = left_property_->getInt();
  int top = top_property_->getInt();

  // Update panel position and dimensions
  panel_->setDimensions(static_cast<Ogre::Real>(width), static_cast<Ogre::Real>(height));
  panel_->setPosition(static_cast<Ogre::Real>(left), static_cast<Ogre::Real>(top));

  // Recreate texture if dimensions changed
  if (texture_image_.width() != width || texture_image_.height() != height) {
    createTexture();
  }

  needs_redraw_ = true; // Trigger redraw with new properties

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "BatteryStateDisplay: Updated properties: pos=(" << left << "," << top <<
      "), size=(" << width << "," << height << ")");
}

void BatteryStateDisplay::createTexture()
{
  int width = width_property_->getInt();
  int height = height_property_->getInt();

  if (width <= 0 || height <= 0) {
    RVIZ_COMMON_LOG_WARNING("BatteryStateDisplay: Invalid dimensions, cannot create texture.");
    return;
  }

  // Remove previous texture if it exists and has the same name
  if (!texture_.isNull() && texture_->getName() == texture_name_) {
    Ogre::TextureManager::getSingleton().remove(texture_name_);
    texture_.setNull();
  }

  // Create new Ogre texture
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    texture_name_,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D,
    width, height, 0, Ogre::PF_A8R8G8B8, // ARGB format for QImage
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  // Update material to use the new texture
  if (material_->getTechnique(0)->getPass(0)->getNumTextureUnitStates() > 0) {
    material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTexture(texture_);
  } else {
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
  }

  // Resize QImage buffer
  texture_image_ = QImage(width, height, QImage::Format_ARGB32);
  needs_redraw_ = true; // Ensure it gets drawn after creation
  RVIZ_COMMON_LOG_DEBUG_STREAM("BatteryStateDisplay: Created texture " << texture_name_);
}

void BatteryStateDisplay::updateOverlayTexture()
{
  if (texture_.isNull() || texture_image_.isNull()) {
    RVIZ_COMMON_LOG_WARNING("BatteryStateDisplay: Texture or QImage not ready, skipping redraw.");
    return;
  }

  // --- Drawing Logic ---
  texture_image_.fill(Qt::transparent); // Clear with transparency

  QPainter painter(&texture_image_);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QColor frame_color = frame_color_property_->getColor();
  int width = texture_image_.width();
  int height = texture_image_.height();

  // Calculate percentage (0-100V range)
  float voltage = std::max(min_voltage_, std::min(max_voltage_, current_voltage_));
  float percentage = (voltage - min_voltage_) / (max_voltage_ - min_voltage_);

  // Calculate bar width
  int bar_width = std::max(0, static_cast<int>((width - 4) * percentage)); // Start from 0 width

  // Draw frame
  painter.setPen(QPen(frame_color, 2));
  painter.setBrush(Qt::transparent);
  painter.drawRect(0, 0, width - 1, height - 1);

  // Calculate bar color (Linear interpolation: Red -> Yellow -> Green)
  QColor bar_color;
  if (percentage < 0.5f) {
    // Red to Yellow (0.0 to 0.5)
    bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0);
  } else {
    // Yellow to Green (0.5 to 1.0)
    bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0);
  }

  // Draw bar (only if width > 0)
  if (bar_width > 0) {
      painter.setPen(Qt::NoPen);
      painter.setBrush(bar_color);
      painter.drawRect(2, 2, bar_width, height - 4);
  }

  // Draw voltage text (optional, centered)
  // painter.setPen(Qt::white);
  // painter.drawText(QRect(0, 0, width, height), Qt::AlignCenter,
  //                 QString("%1V").arg(current_voltage_, 0, 'f', 1));

  painter.end(); // Finish painting

  // --- Upload to Ogre Texture ---
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox & pixel_box = pixel_buffer->getCurrentLock();
  uint8_t * pDest = static_cast<uint8_t *>(pixel_box.data);
  size_t ogre_bytes_per_line = pixel_box.rowPitch * Ogre::PixelUtil::getNumElemBytes(pixel_box.format);

  // Copy data, handling potential pitch differences
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

  RVIZ_COMMON_LOG_DEBUG("BatteryStateDisplay: Overlay texture updated.");
}

} // namespace wifi_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wifi_viz::BatteryStateDisplay, rviz_common::Display)