#include "wifi_viz/wifi_state_display.hpp"

#include <algorithm> // For std::max, std::min
#include <chrono>    // Add chrono include

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
#include <QFontMetrics> // Add QFontMetrics include

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
  text_height_(0),
  min_text_width_(0), // Initialize
  max_text_width_(0), // Initialize
  text_margin_(5),    // Initialize margin
  show_critical_flash_(true) // Initialize flash state
{
  static int instance_count = 0;
  instance_count++;

  overlay_name_ = "WifiVizOverlay" + Ogre::StringConverter::toString(instance_count);
  panel_name_ = "WifiVizPanel" + Ogre::StringConverter::toString(instance_count);
  material_name_ = "WifiVizMaterial" + Ogre::StringConverter::toString(instance_count);
  texture_name_ = "WifiVizTexture" + Ogre::StringConverter::toString(instance_count);

  width_property_ = new rviz_common::properties::IntProperty(
    "Bar Width/Height", 200, "Width (Horizontal) or Height (Vertical) of the bar graph itself in pixels.",
    this, SLOT(updateProperties()), this); // Renamed description slightly
  width_property_->setMin(10);

  height_property_ = new rviz_common::properties::IntProperty(
    "Bar Height/Width", 20, "Height (Horizontal) or Width (Vertical) of the bar graph itself in pixels.",
    this, SLOT(updateProperties()), this); // Renamed description slightly
  height_property_->setMin(5);

  left_property_ = new rviz_common::properties::IntProperty(
    "Left", 20, "Pixels from the left edge of the screen for the top-left corner.",
    this, SLOT(updateProperties()), this);
  left_property_->setMin(0);

  top_property_ = new rviz_common::properties::IntProperty(
    "Top", 20, "Pixels from the top edge of the screen for the top-left corner.",
    this, SLOT(updateProperties()), this);
  top_property_->setMin(0);

  frame_color_property_ = new rviz_common::properties::ColorProperty(
    "Frame Color", QColor(255, 255, 255), "Color of the bar's frame.",
    this, SLOT(updateProperties()), this);

  text_color_property_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(255, 255, 255), "Color of the text labels.",
    this, SLOT(updateProperties()), this);

  font_size_property_ = new rviz_common::properties::IntProperty(
    "Font Size", 12, "Font size for the text labels.",
    this, SLOT(updateProperties()), this);
  font_size_property_->setMin(5);

  // Initialize orientation property
  vertical_mode_property_ = new rviz_common::properties::BoolProperty(
    "Vertical Mode", false, "Display the bar vertically instead of horizontally.",
    this, SLOT(updateProperties()), this);

  // Initialize flash timer
  last_flash_time_ = std::chrono::steady_clock::now();
}

WifiStateDisplay::~WifiStateDisplay()
{
  // Clean up Ogre resources
  try {
    if (overlay_) {
      Ogre::OverlayManager::getSingleton().destroy(overlay_name_);
      overlay_ = nullptr; // Set pointer to null after destruction
      panel_ = nullptr; // Panel is destroyed with overlay
    }
    if (Ogre::MaterialManager::getSingleton().resourceExists(material_name_)) {
      Ogre::MaterialManager::getSingleton().remove(material_name_);
    }
    // TexturePtr will handle its own deletion via reference counting,
    // but explicitly removing ensures it's gone if needed.
    if (texture_ && Ogre::TextureManager::getSingleton().resourceExists(texture_name_)) {
       Ogre::TextureManager::getSingleton().remove(texture_name_);
    }
    texture_.reset(); // Release smart pointer reference
    material_.reset(); // Release smart pointer reference

  } catch (Ogre::Exception& e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error destroying Ogre overlay resources: " << e.getDescription());
  }
  // Base class destructor will handle property cleanup
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
  BaseDisplayClass::update(wall_dt, ros_dt);

  // --- Handle Flashing Animation Timer ---
  if (last_msg_) {
    bool is_critical = last_msg_->critical_if_under ?
                       last_msg_->current < last_msg_->critical_value :
                       last_msg_->current > last_msg_->critical_value;

    if (is_critical && last_msg_->critical_animation_type == wifi_viz::msg::MinMaxCurr::ANIMATION_FLASH) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_flash_time_);
      if (elapsed.count() > 500) { // Flash interval (500ms)
        show_critical_flash_ = !show_critical_flash_;
        last_flash_time_ = now;
        needs_redraw_ = true; // Trigger redraw to show/hide
      }
    } else {
      // Ensure flash is reset to visible if not critical or not flashing type
      if (!show_critical_flash_) {
          show_critical_flash_ = true;
          needs_redraw_ = true;
      }
    }
  }
  // --- End Flashing Logic ---

  // Check if the overlay needs to be redrawn (flag set by processMessage, updateProperties, or flash timer)
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
  last_msg_.reset(); // Clear the last message
  show_critical_flash_ = true; // Reset flash state
  needs_redraw_ = true;
}

void WifiStateDisplay::processMessage(wifi_viz::msg::MinMaxCurr::ConstSharedPtr msg)
{
  last_msg_ = msg; // Store the entire message

  needs_redraw_ = true;
}

void WifiStateDisplay::updateProperties()
{
  ensureOverlay();

  if (!overlay_ || !panel_) {
    RVIZ_COMMON_LOG_WARNING("WifiStateDisplay: Overlay not ready, cannot update properties.");
    return;
  }

  // --- Get Properties ---
  bool vertical_mode = vertical_mode_property_->getBool();
  int prop_width = width_property_->getInt();   // Meaning depends on mode
  int prop_height = height_property_->getInt(); // Meaning depends on mode
  int left = left_property_->getInt();
  int top = top_property_->getInt();
  int font_size = font_size_property_->getInt();

  // --- Calculate Text Dimensions (Independent of orientation) ---
  QFont font = QFont();
  font.setPointSize(font_size);
  QFontMetrics fm(font);
  int estimated_text_width = fm.horizontalAdvance(QString("-999.9")); // Width for min/max
  min_text_width_ = estimated_text_width;
  max_text_width_ = estimated_text_width;
  text_height_ = fm.height() + 4; // Height for topic text (if horizontal) or width (if vertical)

  // --- Calculate Bar and Total Dimensions based on Orientation ---
  int bar_width, bar_height, total_width, total_height;

  if (vertical_mode) {
    bar_width = prop_height; // Height property controls width in vertical mode
    bar_height = prop_width; // Width property controls height in vertical mode
    // Total width: Bar width + space for rotated topic text
    total_width = bar_width + text_height_; // text_height_ is topic text height (now width)
    // Total height: Bar height + space for min/max text above/below
    total_height = bar_height + min_text_width_ + max_text_width_ + 2 * text_margin_; // Using text width as height
  } else {
    // Horizontal mode (as before)
    bar_width = prop_width;
    bar_height = prop_height;
    total_width = min_text_width_ + bar_width + max_text_width_ + 2 * text_margin_;
    total_height = bar_height + text_height_;
  }

  // --- Update Panel ---
  try {
    panel_->setDimensions(static_cast<Ogre::Real>(total_width), static_cast<Ogre::Real>(total_height));
    panel_->setPosition(static_cast<Ogre::Real>(left), static_cast<Ogre::Real>(top));
  } catch (Ogre::Exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Error setting panel dimensions/position: " << e.getDescription());
      return;
  }

  // --- Check if Texture Recreation Needed ---
  bool size_changed = (!texture_ ||
                       texture_->getWidth() != (unsigned int)total_width ||
                       texture_->getHeight() != (unsigned int)total_height);

  if (size_changed) {
    createTexture();
  } else {
    needs_redraw_ = true;
  }

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "WifiStateDisplay: Updated properties: mode=" << (vertical_mode ? "Vertical" : "Horizontal") <<
      ", pos=(" << left << "," << top <<
      "), total_size=(" << total_width << "," << total_height << ")" <<
      ", bar_size=(" << bar_width << "," << bar_height << ")");
}

void WifiStateDisplay::createTexture()
{
  ensureOverlay();

  if (!material_) {
      RVIZ_COMMON_LOG_ERROR("WifiStateDisplay: Material is null, cannot create texture.");
      return;
  }

  // --- Recalculate Total Dimensions based on current properties ---
  bool vertical_mode = vertical_mode_property_->getBool();
  int prop_width = width_property_->getInt();
  int prop_height = height_property_->getInt();
  QFont font = QFont();
  font.setPointSize(font_size_property_->getInt());
  QFontMetrics fm(font);
  int estimated_text_width = fm.horizontalAdvance(QString("-999.9"));
  min_text_width_ = estimated_text_width;
  max_text_width_ = estimated_text_width;
  text_height_ = fm.height() + 4;

  int bar_width, bar_height, total_width, total_height;
  if (vertical_mode) {
    bar_width = prop_height;
    bar_height = prop_width;
    total_width = bar_width + text_height_;
    total_height = bar_height + min_text_width_ + max_text_width_ + 2 * text_margin_;
  } else {
    bar_width = prop_width;
    bar_height = prop_height;
    total_width = min_text_width_ + bar_width + max_text_width_ + 2 * text_margin_;
    total_height = bar_height + text_height_;
  }
  // --- End Recalculation ---

  if (total_width <= 0 || total_height <= 0) {
    RVIZ_COMMON_LOG_WARNING("WifiStateDisplay: Invalid dimensions, cannot create texture.");
    return;
  }

  try {
    if (texture_ && Ogre::TextureManager::getSingleton().resourceExists(texture_name_)) {
        Ogre::TextureManager::getSingleton().remove(texture_name_);
    }
    texture_.reset();

    // Create texture with total dimensions
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      total_width, total_height, 0, Ogre::PF_A8R8G8B8,
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
    if (pass->getNumTextureUnitStates() > 0) {
      pass->getTextureUnitState(0)->setTexture(texture_);
    } else {
      pass->createTextureUnitState(texture_name_);
    }
    pass->getTextureUnitState(0)->setTextureFiltering(Ogre::TFO_NONE);

    // Resize QImage with total dimensions
    texture_image_ = QImage(total_width, total_height, QImage::Format_ARGB32);
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

  // --- Declare variables needed later, outside the conditional blocks ---
  bool vertical_mode = false;
  QColor text_color = Qt::white;
  int font_size = 12;
  QColor frame_color = Qt::white;
  int prop_width = 0;
  int prop_height = 0;
  int total_texture_width = 0;
  int total_texture_height = 0;
  int bar_width = 0;
  int bar_height = 0;
  float value_range = 0.0f;
  float clamped_value = 0.0f;
  float percentage = 0.0f;
  int frame_thickness = 2;
  QString title_text = "";
  uint8_t display_precision = 1; // Default precision

  // --- Determine Critical State and Background ---
  QColor background_color = Qt::transparent; // Default background
  bool is_critical = false;
  bool draw_content = true; // Flag to control if content is drawn (for flashing)

  if (last_msg_) {
    is_critical = last_msg_->critical_if_under ?
                  last_msg_->current < last_msg_->critical_value :
                  last_msg_->current > last_msg_->critical_value;

    if (is_critical) {
      if (last_msg_->critical_animation_type == wifi_viz::msg::MinMaxCurr::ANIMATION_COLORIZE) {
        background_color = QColor(
          static_cast<int>(last_msg_->critical_color.r * 255.0),
          static_cast<int>(last_msg_->critical_color.g * 255.0),
          static_cast<int>(last_msg_->critical_color.b * 255.0),
          static_cast<int>(last_msg_->critical_color.a * 255.0) // Use alpha from message
        );
      } else if (last_msg_->critical_animation_type == wifi_viz::msg::MinMaxCurr::ANIMATION_FLASH) {
        if (show_critical_flash_) {
          background_color = QColor(
            static_cast<int>(last_msg_->critical_color.r * 255.0),
            static_cast<int>(last_msg_->critical_color.g * 255.0),
            static_cast<int>(last_msg_->critical_color.b * 255.0),
            static_cast<int>(last_msg_->critical_color.a * 255.0) // Use alpha from message
          );
        } else {
          // If flashing and 'off', don't draw content
          draw_content = false;
        }
      }
      // else ANIMATION_NONE: background remains transparent (or default)
    }
    // else not critical: background remains transparent (or default)
    display_precision = last_msg_->precision; // Get precision from message
  } else {
    // If no message, don't draw content
    draw_content = false;
  }

  // --- Start Drawing ---
  texture_image_.fill(Qt::transparent); // Clear previous frame first

  QPainter painter(&texture_image_);
  painter.setRenderHint(QPainter::Antialiasing, true);

  // --- Draw Background ---
  painter.fillRect(texture_image_.rect(), background_color);

  // --- Draw Content (only if needed) ---
  if (draw_content && last_msg_) { // Ensure last_msg_ is valid here
    // --- Get Properties ---
    vertical_mode = vertical_mode_property_->getBool();
    text_color = text_color_property_->getColor();
    font_size = font_size_property_->getInt();
    QFont font = painter.font();
    font.setPointSize(font_size);
    painter.setFont(font);

    frame_color = frame_color_property_->getColor();
    prop_width = width_property_->getInt();
    prop_height = height_property_->getInt();
    total_texture_width = texture_image_.width();
    total_texture_height = texture_image_.height();

    // --- Calculate Bar Dimensions ---
    bar_width = vertical_mode ? prop_height : prop_width;
    bar_height = vertical_mode ? prop_width : prop_height;

    // --- Calculate Percentage (using last_msg_) ---
    value_range = last_msg_->max - last_msg_->min;
    clamped_value = std::max(last_msg_->min, std::min(last_msg_->max, last_msg_->current));
    percentage = (value_range > 1e-6) ? (clamped_value - last_msg_->min) / value_range : 0.0f;
    // frame_thickness = 2; // Already declared above

    // --- Define Drawing Areas and Draw based on Mode ---
    painter.setPen(text_color); // Default pen for text

    if (vertical_mode) {
      // --- Vertical Layout ---
      // Define Rects (Max at top, Min at bottom)
      QRect max_text_area(0, 0, bar_width, min_text_width_); // Use text width as height
      QRect bar_rect(0, max_text_area.bottom() + text_margin_, bar_width, bar_height);
      QRect min_text_area(0, bar_rect.bottom() + text_margin_, bar_width, max_text_width_);
      QRect topic_text_area(bar_width, 0, text_height_, total_texture_height); // Rotated text area

      // --- Draw Rotated Min/Max Text ---
      painter.setPen(text_color);
      QString max_text = QString::number(last_msg_->max, 'f', display_precision); // Use precision
      QString min_text = QString::number(last_msg_->min, 'f', display_precision); // Use precision

      // Draw Max Text (Rotated -90 deg, centered above bar)
      painter.save();
      painter.translate(max_text_area.center().x(), max_text_area.bottom() - (text_margin_ + 10)); // Added + 10
      painter.rotate(-90);
      QRect rotated_max_rect(-max_text_area.height() / 2, -max_text_area.width() / 2, max_text_area.height(), max_text_area.width());
      painter.drawText(rotated_max_rect, Qt::AlignCenter, max_text);
      painter.restore();

      // Draw Min Text (Rotated -90 deg, centered below bar)
      painter.save();
      painter.translate(min_text_area.center().x(), min_text_area.top() + (text_margin_ + 10)); // Added + 10
      painter.rotate(-90);
      QRect rotated_min_rect(-min_text_area.height() / 2, -min_text_area.width() / 2, min_text_area.height(), min_text_area.width());
      painter.drawText(rotated_min_rect, Qt::AlignCenter, min_text);
      painter.restore();

      // --- Draw Bar (Vertical) ---
      int available_height = bar_rect.height() - 2 * frame_thickness;
      int bar_fill_height = std::max(0, static_cast<int>(available_height * percentage));

      // Draw Frame
      painter.setPen(QPen(frame_color, frame_thickness));
      painter.setBrush(Qt::transparent);
      QRectF frame_draw_rect(
          bar_rect.left() + frame_thickness / 2.0,
          bar_rect.top() + frame_thickness / 2.0,
          bar_rect.width() - frame_thickness,
          bar_rect.height() - frame_thickness);
      painter.drawRect(frame_draw_rect);

      // Draw Fill (Starts from bottom)
      QColor bar_color;
      if (last_msg_->current_color.a > 0.01) { // Check if alpha is significant
          bar_color = QColor(
              static_cast<int>(last_msg_->current_color.r * 255.0),
              static_cast<int>(last_msg_->current_color.g * 255.0),
              static_cast<int>(last_msg_->current_color.b * 255.0),
              static_cast<int>(last_msg_->current_color.a * 255.0)
          );
      } else { // Fallback to gradient
          if (percentage < 0.5f) { bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0); }
          else { bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0); }
      }

      if (bar_fill_height > 0) {
          painter.setPen(Qt::NoPen);
          painter.setBrush(bar_color);
          QRect bar_fill_rect(
              bar_rect.left() + frame_thickness,
              bar_rect.bottom() - frame_thickness - bar_fill_height,
              bar_rect.width() - 2 * frame_thickness,
              bar_fill_height);
          painter.drawRect(bar_fill_rect);
      }

      // --- Draw Rotated Current Value Text (Centered in Bar Rect) ---
      painter.setPen(text_color);
      QString current_text = QString::number(last_msg_->current, 'f', display_precision); // Use precision
      painter.save();
      painter.translate(bar_rect.center());
      painter.rotate(-90);
      QRect rotated_curr_rect(-bar_rect.height() / 2, -bar_rect.width() / 2, bar_rect.height(), bar_rect.width());
      painter.drawText(rotated_curr_rect, Qt::AlignCenter, current_text);
      painter.restore();

      // --- Draw Rotated Topic Text ---
      title_text = QString::fromStdString(last_msg_->title);
      if (title_text.isEmpty()) {
          title_text = QString::fromStdString(topic_property_->getTopicStd());
      }
      if (!title_text.isEmpty()) {
          painter.save();
          painter.translate(topic_text_area.left(), topic_text_area.bottom());
          painter.rotate(-90);
          QRect rotated_draw_rect(0, 0, topic_text_area.height(), topic_text_area.width());
          painter.drawText(rotated_draw_rect.adjusted(2, 0, -2, 0), Qt::AlignCenter, title_text);
          painter.restore();
      }

    } else {
      // --- Horizontal Layout ---
      QRect min_text_rect(0, 0, min_text_width_, bar_height);
      QRect bar_rect(min_text_width_ + text_margin_, 0, bar_width, bar_height);
      QRect max_text_rect(bar_rect.right() + text_margin_, 0, max_text_width_, bar_height);
      QRect topic_text_rect(0, bar_height, total_texture_width, text_height_);

      // Draw Min/Max Text
      QString min_text = QString::number(last_msg_->min, 'f', display_precision); // Use precision
      painter.drawText(min_text_rect, Qt::AlignRight | Qt::AlignVCenter, min_text);
      QString max_text = QString::number(last_msg_->max, 'f', display_precision); // Use precision
      painter.drawText(max_text_rect, Qt::AlignLeft | Qt::AlignVCenter, max_text);

      // Draw Bar Graph
      int available_width = bar_rect.width() - 2 * frame_thickness;
      int bar_fill_width = std::max(0, static_cast<int>(available_width * percentage));

      // Draw Frame
      painter.setPen(QPen(frame_color, frame_thickness));
      painter.setBrush(Qt::transparent);
      QRectF frame_draw_rect(
          bar_rect.left() + frame_thickness / 2.0,
          bar_rect.top() + frame_thickness / 2.0,
          bar_rect.width() - frame_thickness,
          bar_rect.height() - frame_thickness);
      painter.drawRect(frame_draw_rect);

      // Draw Fill (Using color from message if available, otherwise default gradient)
      QColor bar_color;
      if (last_msg_->current_color.a > 0.01) { // Check if alpha is significant
          bar_color = QColor(
              static_cast<int>(last_msg_->current_color.r * 255.0),
              static_cast<int>(last_msg_->current_color.g * 255.0),
              static_cast<int>(last_msg_->current_color.b * 255.0),
              static_cast<int>(last_msg_->current_color.a * 255.0)
          );
      } else { // Fallback to gradient
          if (percentage < 0.5f) { bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0); }
          else { bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0); }
      }

      if (bar_fill_width > 0) {
          painter.setPen(Qt::NoPen);
          painter.setBrush(bar_color);
          QRect bar_fill_rect(
              bar_rect.left() + frame_thickness,
              bar_rect.top() + frame_thickness,
              bar_fill_width,
              bar_rect.height() - 2 * frame_thickness);
          painter.drawRect(bar_fill_rect);
      }

      // Draw Current Value Text
      painter.setPen(text_color);
      QString current_text = QString::number(last_msg_->current, 'f', display_precision); // Use precision
      painter.drawText(bar_rect, Qt::AlignCenter, current_text);

      // Draw Topic Text (Using title from message if available, else topic property)
      title_text = QString::fromStdString(last_msg_->title);
      if (title_text.isEmpty()) {
          title_text = QString::fromStdString(topic_property_->getTopicStd()); // Fallback to topic
      }
      if (!title_text.isEmpty()) {
          painter.drawText(topic_text_rect.adjusted(0, 2, 0, -2),
                           Qt::AlignCenter | Qt::AlignTop,
                           title_text);
      }
    }
  } // End if(draw_content && last_msg_)

  painter.end();

  // --- Upload Texture Data to Ogre (Always happens) ---
  try {
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox & pixel_box = pixel_buffer->getCurrentLock();
    uint8_t * pDest = static_cast<uint8_t *>(pixel_box.data);
    size_t ogre_bytes_per_line = pixel_box.rowPitch * Ogre::PixelUtil::getNumElemBytes(pixel_box.format);

    if (ogre_bytes_per_line == static_cast<size_t>(texture_image_.bytesPerLine())) {
      memcpy(pDest, texture_image_.constBits(), texture_image_.sizeInBytes());
    } else {
      for (int y = 0; y < texture_image_.height(); ++y) {
        memcpy(
          pDest + y * ogre_bytes_per_line,
          texture_image_.constScanLine(y),
          texture_image_.bytesPerLine());
      }
    }
    pixel_buffer->unlock();
  } catch (Ogre::Exception & e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error uploading texture data: " << e.getDescription());
  }
}

} // namespace wifi_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wifi_viz::WifiStateDisplay, rviz_common::Display)
