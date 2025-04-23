#include "min_max_curr_rviz_overlay/min_max_curr_display.hpp"

#include <algorithm> // For std::max, std::min
#include <chrono>    // Add chrono include
#include <sstream>   // For string stream (JSON conversion)
#include <iomanip>   // For std::fixed, std::setprecision

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
#include <rviz_common/properties/string_property.hpp> // Include StringProperty header

// Custom Message Header
#include "min_max_curr_rviz_overlay/msg/min_max_curr.hpp"
#include "min_max_curr_rviz_overlay/srv/trigger_critical_action.hpp" // Include service header

namespace min_max_curr_rviz_overlay
{

using BaseDisplayClass = rviz_common::RosTopicDisplay<min_max_curr_rviz_overlay::msg::MinMaxCurr>;
using TriggerCriticalAction = min_max_curr_rviz_overlay::srv::TriggerCriticalAction; // Alias for service type

MinMaxCurrDisplay::MinMaxCurrDisplay()
: BaseDisplayClass(),
  overlay_(nullptr),
  panel_(nullptr),
  needs_redraw_(false),
  text_height_(0),
  min_text_width_(0), // Initialize
  max_text_width_(0), // Initialize
  topic_text_width_(0), // Initialize topic text width
  text_margin_(5),    // Initialize margin
  show_critical_flash_(true), // Initialize flash state
  critical_service_client_(nullptr), // Initialize client pointer
  critical_service_pending_(false)
{
  static int instance_count = 0;
  instance_count++;

  overlay_name_ = "MinMaxCurrOverlay" + Ogre::StringConverter::toString(instance_count);
  panel_name_ = "MinMaxCurrPanel" + Ogre::StringConverter::toString(instance_count);
  material_name_ = "MinMaxCurrMaterial" + Ogre::StringConverter::toString(instance_count);
  texture_name_ = "MinMaxCurrTexture" + Ogre::StringConverter::toString(instance_count);

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

  // Add Critical Service Name Property
  critical_service_name_property_ = new rviz_common::properties::StringProperty(
    "Critical Service Name", "",
    "Name of the TriggerCriticalAction service to call when value is critical. Leave empty to disable.",
    this, SLOT(updateCriticalService()), this);

  // Initialize flash timer
  last_flash_time_ = std::chrono::steady_clock::now();
}

MinMaxCurrDisplay::~MinMaxCurrDisplay()
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

void MinMaxCurrDisplay::onInitialize()
{
  BaseDisplayClass::onInitialize();

  // Create the ROS 2 node needed for the service client
  // The node is managed by the DisplayContext
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", "No topic selected");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Critical Service", "No service configured."); // Initial status

  ensureOverlay();
  updateCriticalService(); // Create client based on initial property value
  updateProperties();
  RVIZ_COMMON_LOG_INFO("MinMaxCurrDisplay: Initialized.");
}

void MinMaxCurrDisplay::ensureOverlay()
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
    RVIZ_COMMON_LOG_INFO_STREAM("MinMaxCurrDisplay: Created Ogre overlay '" << overlay_name_ << "'.");

  } catch (Ogre::Exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Error creating Ogre overlay: " << e.getDescription());
      overlay_ = nullptr;
      panel_ = nullptr;
  }
}

void MinMaxCurrDisplay::onEnable()
{
  BaseDisplayClass::onEnable();
  // Show overlay when enabled
  if (overlay_) {
    overlay_->show();
  }
  needs_redraw_ = true; // Trigger redraw on enable
}

void MinMaxCurrDisplay::onDisable()
{
  BaseDisplayClass::onDisable();
  // Hide overlay when disabled
  if (overlay_) {
    overlay_->hide();
  }
}

void MinMaxCurrDisplay::update(float wall_dt, float ros_dt)
{
  // *** Crucial: Call the base class update method ***
  BaseDisplayClass::update(wall_dt, ros_dt);

  bool needs_service_call = false; // Flag to trigger service call outside flash logic

  // --- Handle Flashing Animation Timer & Critical State Check ---
  if (last_msg_) {
    bool is_critical = last_msg_->critical_if_under ?
                       last_msg_->current < last_msg_->critical_value :
                       last_msg_->current > last_msg_->critical_value;

    if (is_critical) {
        // Mark that a service call might be needed if configured
        needs_service_call = true;

        // Handle flashing animation
        if (last_msg_->critical_animation_type == min_max_curr_rviz_overlay::msg::MinMaxCurr::ANIMATION_FLASH) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_flash_time_);
            if (elapsed.count() > 500) { // Flash interval (500ms)
                show_critical_flash_ = !show_critical_flash_;
                last_flash_time_ = now;
                needs_redraw_ = true; // Trigger redraw to show/hide
            }
        } else {
             // Ensure flash is reset to visible if not flashing type but critical
             if (!show_critical_flash_) {
                 show_critical_flash_ = true;
                 needs_redraw_ = true;
             }
        }
    } else {
      // Not critical: Reset flash state and pending service call flag
      if (!show_critical_flash_) {
          show_critical_flash_ = true;
          needs_redraw_ = true;
      }
      critical_service_pending_ = false; // Reset pending flag if no longer critical
    }
  } else {
      // No message: Reset flash state and pending service call flag
      if (!show_critical_flash_) {
          show_critical_flash_ = true;
          needs_redraw_ = true;
      }
      critical_service_pending_ = false;
  }
  // --- End Flashing & Critical Check ---

  // --- Trigger Critical Service Call (if needed and not already pending) ---
  if (needs_service_call && critical_service_client_ && !critical_service_pending_) {
      // Check if service is available (optional, async call handles unavailability)
      // if (!critical_service_client_->service_is_ready()) {
      //     RVIZ_COMMON_LOG_WARNING_STREAM("Critical service '" << critical_service_name_property_->getStdString() << "' not available.");
      //     setStatus(rviz_common::properties::StatusProperty::Warn, "Critical Service", "Service not available.");
      // } else
      {
          setStatus(rviz_common::properties::StatusProperty::Ok, "Critical Service", "Calling service...");
          critical_service_pending_ = true; // Set flag to prevent spamming

          auto request = std::make_shared<TriggerCriticalAction::Request>();
          request->json_data = messageToJson(*last_msg_);

          // Use weak_ptr for safety in async callback
          // Call weak_from_this() as a member function
          auto weak_this = this->weak_from_this();
          critical_service_client_->async_send_request(
              request,
              [weak_this](rclcpp::Client<TriggerCriticalAction>::SharedFuture future) {
                  auto shared_this = weak_this.lock();
                  if (!shared_this) {
                      // Plugin might have been destroyed
                      return;
                  }
                  // Reset pending flag regardless of outcome
                  shared_this->critical_service_pending_ = false;
                  try {
                      auto response = future.get();
                      if (response->success) {
                          RVIZ_COMMON_LOG_INFO_STREAM("Critical service call successful: " << response->message);
                          shared_this->setStatus(rviz_common::properties::StatusProperty::Ok, "Critical Service", "Call successful.");
                      } else {
                          RVIZ_COMMON_LOG_ERROR_STREAM("Critical service call failed: " << response->message);
                          shared_this->setStatus(rviz_common::properties::StatusProperty::Error, "Critical Service", "Call failed: " + QString::fromStdString(response->message));
                      }
                  } catch (const std::exception &e) {
                      RVIZ_COMMON_LOG_ERROR_STREAM("Exception during critical service call: " << e.what());
                      shared_this->setStatus(rviz_common::properties::StatusProperty::Error, "Critical Service", "Call exception.");
                  }
              });
      }
  }

  // Check if the overlay needs to be redrawn
  if (needs_redraw_) {
    if (overlay_ && panel_ && texture_ && !texture_image_.isNull()) {
        updateOverlayTexture();
        needs_redraw_ = false; // Reset the flag after redrawing
    }
  }
}

void MinMaxCurrDisplay::reset()
{
  // Call base class reset
  BaseDisplayClass::reset();
  // Reset internal state and trigger redraw
  last_msg_.reset(); // Clear the last message
  show_critical_flash_ = true; // Reset flash state
  needs_redraw_ = true;
}

void MinMaxCurrDisplay::processMessage(min_max_curr_rviz_overlay::msg::MinMaxCurr::ConstSharedPtr msg)
{
  // Check if layout-affecting properties changed (compact or title)
  // Also check if it's the first message
  bool layout_changed = !last_msg_ ||
                        last_msg_->compact != msg->compact ||
                        (last_msg_->title.empty() && !msg->title.empty() && topic_property_->getTopicStd().empty()) || // Gained a title when none existed (and no topic fallback)
                        (!last_msg_->title.empty() && last_msg_->title != msg->title) || // Title changed
                        (last_msg_->title.empty() && msg->title.empty() && !topic_property_->getTopicStd().empty() && topic_text_width_ == 0); // No title, but topic exists and width wasn't calculated yet

  // --- Update Critical Service Name Property from Message ---
  // Check if the message provides a service name and if it differs from the current property setting
  if (!msg->critical_service_name.empty() &&
      critical_service_name_property_->getStdString() != msg->critical_service_name)
  {
    RVIZ_COMMON_LOG_INFO_STREAM("Updating Critical Service Name property from message: " << msg->critical_service_name);
    // Set the property value. This will automatically trigger the updateCriticalService() slot.
    // Use QVariant to set the StringProperty value.
    critical_service_name_property_->setValue(QString::fromStdString(msg->critical_service_name));
    // Note: updateCriticalService() will handle client creation/resetting.
  }
  // --- End Service Name Update ---

  last_msg_ = msg; // Store the entire message

  // If layout changed, force property update which recalculates dimensions and texture
  if (layout_changed) {
      updateProperties(); // This will trigger texture recreation if needed via size check
  } else {
      needs_redraw_ = true; // Otherwise, just redraw with existing texture size
  }
}

// Helper function to calculate dimensions based on properties and message
void MinMaxCurrDisplay::calculateDimensions(
    int& total_width, int& total_height,
    int& bar_width, int& bar_height,
    int& min_text_w, int& max_text_w, int& topic_text_w, int& text_h)
{
    bool vertical_mode = vertical_mode_property_->getBool();
    int prop_width = width_property_->getInt();
    int prop_height = height_property_->getInt();
    int font_size = font_size_property_->getInt();
    // Use last message for compact flag and title, default to false/empty if no message yet
    bool compact_mode = last_msg_ ? last_msg_->compact : false;
    uint8_t precision = last_msg_ ? last_msg_->precision : 1;
    QString title_str = last_msg_ ? QString::fromStdString(last_msg_->title) : "";
    if (title_str.isEmpty() && topic_property_) { // Check topic_property_ validity
        title_str = QString::fromStdString(topic_property_->getTopicStd());
    }

    QFont font = QFont();
    font.setPointSize(font_size);
    QFontMetrics fm(font);

    // Calculate base text dimensions using precision for better estimate
    QString sample_num_str = QString::number(123.456, 'f', precision); // Sample number
    min_text_w = fm.horizontalAdvance(sample_num_str);
    max_text_w = min_text_w;
    text_h = fm.height() + 4; // Basic text height

    // Calculate actual topic text width
    topic_text_w = fm.horizontalAdvance(title_str);

    // Calculate Bar and Total Dimensions based on Orientation and Compact mode
    if (vertical_mode) {
        bar_width = prop_height; // Height property controls width in vertical mode
        bar_height = prop_width; // Width property controls height in vertical mode

        if (compact_mode) {
            // Compact Vertical: Title (top, horizontal), Max (rot), Bar, Min (rot)
            // Width determined by the wider of the bar or the title
            total_width = std::max(bar_width, topic_text_w + 2 * text_margin_);
            // Height: Title + Max(rot) + Bar + Min(rot) + margins
            total_height = text_h + max_text_w + bar_height + min_text_w + 3 * text_margin_;
        } else {
            // Non-Compact Vertical: [Max(rot), Bar, Min(rot)] | [Title(rot)]
            // Total width: Bar width + space for rotated topic text + margin
            total_width = bar_width + text_h + text_margin_; // text_h is rotated topic text width
            // Total height: Max height of (Bar + rotated Min/Max) or rotated Title
            total_height = std::max(bar_height + max_text_w + min_text_w + 2 * text_margin_, topic_text_w);
        }
    } else { // Horizontal mode
        bar_width = prop_width;
        bar_height = prop_height;

        if (compact_mode) {
            // Compact Horizontal: [Title][Min][Bar][Max]
            total_width = topic_text_w + min_text_w + bar_width + max_text_w + 3 * text_margin_;
            // Height determined by the taller of the bar or the text
            total_height = std::max(bar_height, text_h);
        } else {
            // Non-Compact Horizontal: [Min][Bar][Max] (top row) / [Title] (bottom row)
            // Width determined by top row
            total_width = min_text_w + bar_width + max_text_w + 2 * text_margin_;
            // Height: Bar row + Title row
            total_height = bar_height + text_h;
        }
    }
    // Ensure minimum dimensions
    total_width = std::max(10, total_width);
    total_height = std::max(10, total_height);
}

void MinMaxCurrDisplay::updateProperties()
{
  ensureOverlay();

  if (!overlay_ || !panel_) {
    RVIZ_COMMON_LOG_WARNING("MinMaxCurrDisplay: Overlay not ready, cannot update properties.");
    return;
  }

  // --- Calculate Dimensions using helper ---
  int total_width, total_height, bar_width, bar_height;
  // Use the helper function to get all dimensions based on current props and last_msg_
  calculateDimensions(total_width, total_height, bar_width, bar_height,
                      min_text_width_, max_text_width_, topic_text_width_, text_height_);

  // --- Update Panel ---
  try {
    panel_->setDimensions(static_cast<Ogre::Real>(total_width), static_cast<Ogre::Real>(total_height));
    panel_->setPosition(static_cast<Ogre::Real>(left_property_->getInt()), static_cast<Ogre::Real>(top_property_->getInt()));
  } catch (Ogre::Exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Error setting panel dimensions/position: " << e.getDescription());
      return;
  }

  // --- Check if Texture Recreation Needed ---
  // Texture might be null if creation failed before
  bool size_changed = (!texture_ ||
                       texture_->getWidth() != (unsigned int)total_width ||
                       texture_->getHeight() != (unsigned int)total_height);

  if (size_changed) {
    createTexture(); // This will also set needs_redraw_
  } else {
    needs_redraw_ = true; // Trigger redraw if only properties like color changed, or if layout changed but size didn't
  }

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "MinMaxCurrDisplay: Updated properties: mode=" << (vertical_mode_property_->getBool() ? "Vertical" : "Horizontal") <<
      ", compact=" << (last_msg_ ? last_msg_->compact : false) <<
      ", pos=(" << left_property_->getInt() << "," << top_property_->getInt() <<
      "), total_size=(" << total_width << "," << total_height << ")" <<
      ", bar_size=(" << bar_width << "," << bar_height << ")");
}

void MinMaxCurrDisplay::createTexture()
{
  ensureOverlay();

  if (!material_) {
      RVIZ_COMMON_LOG_ERROR("MinMaxCurrDisplay: Material is null, cannot create texture.");
      return;
  }

  // --- Recalculate Total Dimensions using helper ---
  int total_width, total_height, bar_width, bar_height;
  // Use the helper function - recalculates based on current props and last_msg_
  calculateDimensions(total_width, total_height, bar_width, bar_height,
                      min_text_width_, max_text_width_, topic_text_width_, text_height_);

  // Note: calculateDimensions already ensures minimum 10x10 size.

  try {
    // Remove existing texture if it exists
    if (texture_ && Ogre::TextureManager::getSingleton().resourceExists(texture_name_)) {
        Ogre::TextureManager::getSingleton().remove(texture_name_);
    }
    texture_.reset(); // Release smart pointer reference

    // Create texture with calculated dimensions
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      total_width, total_height, 0, Ogre::PF_A8R8G8B8, // Pixel format ARGB
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    // Ensure material has a technique and pass
     if (material_->getNumTechniques() == 0) {
         Ogre::Technique *tech = material_->createTechnique();
         tech->createPass();
     } else if (material_->getTechnique(0)->getNumPasses() == 0) {
         material_->getTechnique(0)->createPass();
     }

    Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
    // Remove existing texture unit states to avoid conflicts
    pass->removeAllTextureUnitStates();
    // Create new texture unit state for our texture
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState(texture_name_);
    tex_unit->setTextureFiltering(Ogre::TFO_NONE); // No filtering
    // Set up blending for transparency
    tex_unit->setColourOperationEx(Ogre::LBX_MODULATE, Ogre::LBS_TEXTURE, Ogre::LBS_CURRENT); // Modulate with vertex color
    tex_unit->setAlphaOperation(Ogre::LBX_MODULATE, Ogre::LBS_TEXTURE, Ogre::LBS_CURRENT); // Modulate alpha

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(false);
    pass->setLightingEnabled(false); // No lighting needed
    pass->setCullingMode(Ogre::CULL_NONE); // No culling

    // Resize QImage buffer
    texture_image_ = QImage(total_width, total_height, QImage::Format_ARGB32);
    texture_image_.fill(Qt::transparent); // Initialize with transparency

    needs_redraw_ = true; // Mark for redraw

    RVIZ_COMMON_LOG_DEBUG_STREAM("MinMaxCurrDisplay: Created/Recreated texture " << texture_name_ << " with size " << total_width << "x" << total_height);

  } catch (Ogre::Exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Error creating/recreating Ogre texture: " << e.getDescription());
      texture_.reset();
      texture_image_ = QImage(); // Reset QImage as well
  }
}

void MinMaxCurrDisplay::updateOverlayTexture()
{
  if (!texture_ || texture_image_.isNull() || !last_msg_) { // Also check last_msg_ here
    // Don't log warning if simply no message received yet
    if (last_msg_) {
        RVIZ_COMMON_LOG_WARNING("MinMaxCurrDisplay: Texture or QImage not ready, skipping redraw.");
    }
    return;
  }

  // --- Get state from message and properties ---
  bool vertical_mode = vertical_mode_property_->getBool();
  bool compact_mode = last_msg_->compact;
  uint8_t display_precision = last_msg_->precision;
  QColor text_color = text_color_property_->getColor();
  QColor frame_color = frame_color_property_->getColor();
  int font_size = font_size_property_->getInt();
  int frame_thickness = 2; // Could be a property later

  // Determine title text
  QString title_text = QString::fromStdString(last_msg_->title);
  if (title_text.isEmpty() && topic_property_) {
      title_text = QString::fromStdString(topic_property_->getTopicStd());
  }

  // --- Determine Critical State and Background ---
  QColor background_color = Qt::transparent; // Default background
  bool is_critical = false;
  bool draw_content = true; // Flag to control if content is drawn (for flashing)

  is_critical = last_msg_->critical_if_under ?
                last_msg_->current < last_msg_->critical_value :
                last_msg_->current > last_msg_->critical_value;

  if (is_critical) {
    if (last_msg_->critical_animation_type == min_max_curr_rviz_overlay::msg::MinMaxCurr::ANIMATION_COLORIZE) {
      background_color = QColor(
        static_cast<int>(last_msg_->critical_color.r * 255.0),
        static_cast<int>(last_msg_->critical_color.g * 255.0),
        static_cast<int>(last_msg_->critical_color.b * 255.0),
        static_cast<int>(last_msg_->critical_color.a * 255.0)
      );
    } else if (last_msg_->critical_animation_type == min_max_curr_rviz_overlay::msg::MinMaxCurr::ANIMATION_FLASH) {
      if (show_critical_flash_) {
        background_color = QColor(
          static_cast<int>(last_msg_->critical_color.r * 255.0),
          static_cast<int>(last_msg_->critical_color.g * 255.0),
          static_cast<int>(last_msg_->critical_color.b * 255.0),
          static_cast<int>(last_msg_->critical_color.a * 255.0)
        );
      } else {
        draw_content = false; // Don't draw content during the 'off' phase of flash
      }
    }
    // else ANIMATION_NONE: background remains transparent
  }
  // else not critical: background remains transparent

  // --- Start Drawing ---
  texture_image_.fill(Qt::transparent); // Clear previous frame first

  QPainter painter(&texture_image_);
  painter.setRenderHint(QPainter::Antialiasing, true);
  QFont font = painter.font();
  font.setPointSize(font_size);
  painter.setFont(font);
  QFontMetrics fm(painter.font()); // Use painter's font for metrics

  // --- Draw Background ---
  painter.fillRect(texture_image_.rect(), background_color);

  // --- Draw Content (only if needed and message exists) ---
  if (draw_content) {
    // --- Calculate values needed for drawing ---
    QString min_str = QString::number(last_msg_->min, 'f', display_precision);
    QString max_str = QString::number(last_msg_->max, 'f', display_precision);
    QString current_str = QString::number(last_msg_->current, 'f', display_precision);
    int current_min_text_width = fm.horizontalAdvance(min_str);
    int current_max_text_width = fm.horizontalAdvance(max_str);
    int current_topic_text_width = fm.horizontalAdvance(title_text);
    int current_text_height = fm.height(); // Use actual height without padding here

    int bar_draw_width = vertical_mode ? height_property_->getInt() : width_property_->getInt();
    int bar_draw_height = vertical_mode ? width_property_->getInt() : height_property_->getInt();

    float value_range = last_msg_->max - last_msg_->min;
    float clamped_value = std::max(last_msg_->min, std::min(last_msg_->max, last_msg_->current));
    float percentage = (value_range > 1e-6) ? (clamped_value - last_msg_->min) / value_range : 0.0f;

    painter.setPen(text_color); // Default pen for text

    // --- Layout Logic ---
    if (vertical_mode) {
        if (compact_mode) {
            // --- Vertical Compact Layout ---
            // [ Title (centered, top) ]
            // [ Max (Rot) ]
            // [ Bar ]
            // [ Min (Rot) ]
            int content_width = bar_draw_width; // Bar defines the width
            int title_y = text_margin_;
            int max_rot_y = title_y + current_text_height + text_margin_;
            int bar_y = max_rot_y + current_max_text_width + text_margin_; // Rotated max text height = its width
            int min_rot_y = bar_y + bar_draw_height + text_margin_;
            int center_x = texture_image_.width() / 2; // Center horizontally

            // Draw Title (Horizontal, Centered at Top)
            if (!title_text.isEmpty()) {
                QRect topic_text_area(center_x - current_topic_text_width / 2, title_y, current_topic_text_width, current_text_height);
                painter.drawText(topic_text_area, Qt::AlignCenter | Qt::AlignVCenter, title_text);
            }

            // Define areas relative to center_x for rotation
            QRect max_text_area(center_x - content_width / 2, max_rot_y, content_width, current_max_text_width);
            QRect bar_rect(center_x - content_width / 2, bar_y, content_width, bar_draw_height);
            QRect min_text_area(center_x - content_width / 2, min_rot_y, content_width, current_min_text_width);

            // Draw Rotated Max Text
            painter.save();
            painter.translate(max_text_area.center().x(), max_text_area.center().y());
            painter.rotate(-90);
            QRect rotated_max_rect(-max_text_area.height() / 2, -max_text_area.width() / 2, max_text_area.height(), max_text_area.width());
            painter.drawText(rotated_max_rect, Qt::AlignCenter, max_str);
            painter.restore();

            // Draw Rotated Min Text
            painter.save();
            painter.translate(min_text_area.center().x(), min_text_area.center().y());
            painter.rotate(-90);
            QRect rotated_min_rect(-min_text_area.height() / 2, -min_text_area.width() / 2, min_text_area.height(), min_text_area.width());
            painter.drawText(rotated_min_rect, Qt::AlignCenter, min_str);
            painter.restore();

            // Draw Bar (Vertical)
            int available_height = bar_rect.height() - 2 * frame_thickness;
            int bar_fill_height = std::max(0, static_cast<int>(available_height * percentage));
            // Draw Frame
            painter.setPen(QPen(frame_color, frame_thickness));
            painter.setBrush(Qt::transparent);
            QRectF frame_draw_rect(bar_rect.left() + frame_thickness / 2.0, bar_rect.top() + frame_thickness / 2.0, bar_rect.width() - frame_thickness, bar_rect.height() - frame_thickness);
            painter.drawRect(frame_draw_rect);
            // Draw Fill
            QColor bar_color;
             if (last_msg_->current_color.a > 0.01) {
                 bar_color = QColor( static_cast<int>(last_msg_->current_color.r * 255.0), static_cast<int>(last_msg_->current_color.g * 255.0), static_cast<int>(last_msg_->current_color.b * 255.0), static_cast<int>(last_msg_->current_color.a * 255.0) );
             } else {
                 if (percentage < 0.5f) { bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0); } else { bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0); }
             }
            if (bar_fill_height > 0) {
                painter.setPen(Qt::NoPen);
                painter.setBrush(bar_color);
                QRect bar_fill_rect(bar_rect.left() + frame_thickness, bar_rect.bottom() - frame_thickness - bar_fill_height, bar_rect.width() - 2 * frame_thickness, bar_fill_height);
                painter.drawRect(bar_fill_rect);
            }

            // Draw Rotated Current Value Text (Centered in Bar Rect)
            painter.setPen(text_color);
            painter.save();
            painter.translate(bar_rect.center());
            painter.rotate(-90);
            QRect rotated_curr_rect(-bar_rect.height() / 2, -bar_rect.width() / 2, bar_rect.height(), bar_rect.width());
            painter.drawText(rotated_curr_rect, Qt::AlignCenter, current_str);
            painter.restore();

        } else {
            // --- Vertical Non-Compact Layout (Original Logic) ---
            int bar_area_height = bar_draw_height + current_max_text_width + current_min_text_width + 2 * text_margin_;
            int title_area_width = current_text_height; // Rotated title width = text height
            int bar_area_width = bar_draw_width;
            int total_draw_height = std::max(bar_area_height, current_topic_text_width); // Max height of bar section or rotated title
            int bar_area_y_offset = (total_draw_height - bar_area_height) / 2; // Center bar area vertically if needed
            int title_area_x_offset = bar_area_width + text_margin_;

            // Define Rects
            QRect max_text_area(0, bar_area_y_offset, bar_area_width, current_max_text_width);
            QRect bar_rect(0, max_text_area.bottom() + text_margin_, bar_area_width, bar_draw_height);
            QRect min_text_area(0, bar_rect.bottom() + text_margin_, bar_area_width, current_min_text_width);
            QRect topic_text_area(title_area_x_offset, 0, title_area_width, total_draw_height);

            // Draw Rotated Min/Max Text
            painter.setPen(text_color);
            painter.save(); // Max
            painter.translate(max_text_area.center().x(), max_text_area.center().y());
            painter.rotate(-90);
            QRect rotated_max_rect(-max_text_area.height() / 2, -max_text_area.width() / 2, max_text_area.height(), max_text_area.width());
            painter.drawText(rotated_max_rect, Qt::AlignCenter, max_str);
            painter.restore();
            painter.save(); // Min
            painter.translate(min_text_area.center().x(), min_text_area.center().y());
            painter.rotate(-90);
            QRect rotated_min_rect(-min_text_area.height() / 2, -min_text_area.width() / 2, min_text_area.height(), min_text_area.width());
            painter.drawText(rotated_min_rect, Qt::AlignCenter, min_str);
            painter.restore();

            // Draw Bar (Vertical)
            int available_height = bar_rect.height() - 2 * frame_thickness;
            int bar_fill_height = std::max(0, static_cast<int>(available_height * percentage));
            // Draw Frame
            painter.setPen(QPen(frame_color, frame_thickness));
            painter.setBrush(Qt::transparent);
            QRectF frame_draw_rect(bar_rect.left() + frame_thickness / 2.0, bar_rect.top() + frame_thickness / 2.0, bar_rect.width() - frame_thickness, bar_rect.height() - frame_thickness);
            painter.drawRect(frame_draw_rect);
            // Draw Fill
            QColor bar_color;
             if (last_msg_->current_color.a > 0.01) {
                 bar_color = QColor( static_cast<int>(last_msg_->current_color.r * 255.0), static_cast<int>(last_msg_->current_color.g * 255.0), static_cast<int>(last_msg_->current_color.b * 255.0), static_cast<int>(last_msg_->current_color.a * 255.0) );
             } else {
                 if (percentage < 0.5f) { bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0); } else { bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0); }
             }
            if (bar_fill_height > 0) {
                painter.setPen(Qt::NoPen);
                painter.setBrush(bar_color);
                QRect bar_fill_rect(bar_rect.left() + frame_thickness, bar_rect.bottom() - frame_thickness - bar_fill_height, bar_rect.width() - 2 * frame_thickness, bar_fill_height);
                painter.drawRect(bar_fill_rect);
            }

            // Draw Rotated Current Value Text
            painter.setPen(text_color);
            painter.save();
            painter.translate(bar_rect.center());
            painter.rotate(-90);
            QRect rotated_curr_rect(-bar_rect.height() / 2, -bar_rect.width() / 2, bar_rect.height(), bar_rect.width());
            painter.drawText(rotated_curr_rect, Qt::AlignCenter, current_str);
            painter.restore();

            // Draw Rotated Topic Text
            if (!title_text.isEmpty()) {
                painter.save();
                painter.translate(topic_text_area.left(), topic_text_area.bottom()); // Pivot at bottom-left
                painter.rotate(-90);
                QRect rotated_draw_rect(0, 0, topic_text_area.height(), topic_text_area.width()); // Width/Height swapped
                painter.drawText(rotated_draw_rect.adjusted(2, 0, -2, 0), Qt::AlignCenter, title_text);
                painter.restore();
            }
        }
    } else { // Horizontal Mode
        int content_height = std::max(bar_draw_height, current_text_height); // Height of the main row
        int y_offset = (texture_image_.height() - content_height) / 2; // Center vertically if needed (e.g., non-compact has extra space)

        if (compact_mode) {
            // --- Horizontal Compact Layout ---
            // [Title][Min][Bar][Max] - All vertically centered in texture height
            int title_x = text_margin_;
            int min_x = title_x + current_topic_text_width + text_margin_;
            int bar_x = min_x + current_min_text_width + text_margin_;
            int max_x = bar_x + bar_draw_width + text_margin_;

            QRect topic_text_rect(title_x, y_offset, current_topic_text_width, content_height);
            QRect min_text_rect(min_x, y_offset, current_min_text_width, content_height);
            QRect bar_rect(bar_x, y_offset + (content_height - bar_draw_height)/2, bar_draw_width, bar_draw_height); // Center bar within content height
            QRect max_text_rect(max_x, y_offset, current_max_text_width, content_height);

            // Draw Title Text (Leftmost)
            if (!title_text.isEmpty()) {
                painter.drawText(topic_text_rect, Qt::AlignLeft | Qt::AlignVCenter, title_text);
            }
            // Draw Min Text
            painter.drawText(min_text_rect, Qt::AlignRight | Qt::AlignVCenter, min_str);
            // Draw Max Text
            painter.drawText(max_text_rect, Qt::AlignLeft | Qt::AlignVCenter, max_str);

            // Draw Bar Graph
            int available_width = bar_rect.width() - 2 * frame_thickness;
            int bar_fill_width = std::max(0, static_cast<int>(available_width * percentage));
            // Draw Frame
            painter.setPen(QPen(frame_color, frame_thickness));
            painter.setBrush(Qt::transparent);
            QRectF frame_draw_rect(bar_rect.left() + frame_thickness / 2.0, bar_rect.top() + frame_thickness / 2.0, bar_rect.width() - frame_thickness, bar_rect.height() - frame_thickness);
            painter.drawRect(frame_draw_rect);
            // Draw Fill
            QColor bar_color;
             if (last_msg_->current_color.a > 0.01) {
                 bar_color = QColor( static_cast<int>(last_msg_->current_color.r * 255.0), static_cast<int>(last_msg_->current_color.g * 255.0), static_cast<int>(last_msg_->current_color.b * 255.0), static_cast<int>(last_msg_->current_color.a * 255.0) );
             } else {
                 if (percentage < 0.5f) { bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0); } else { bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0); }
             }
            if (bar_fill_width > 0) {
                painter.setPen(Qt::NoPen);
                painter.setBrush(bar_color);
                QRect bar_fill_rect(bar_rect.left() + frame_thickness, bar_rect.top() + frame_thickness, bar_fill_width, bar_rect.height() - 2 * frame_thickness);
                painter.drawRect(bar_fill_rect);
            }

            // Draw Current Value Text
            painter.setPen(text_color);
            painter.drawText(bar_rect, Qt::AlignCenter, current_str);

        } else {
            // --- Horizontal Non-Compact Layout (Original Logic) ---
            // [Min][Bar][Max] (Top row)
            // [   Title   ] (Bottom row)
            int bar_row_y = 0; // Top row starts at y=0
            int title_row_y = bar_draw_height; // Title row starts below bar row

            int min_x = text_margin_;
            int bar_x = min_x + current_min_text_width + text_margin_;
            int max_x = bar_x + bar_draw_width + text_margin_;

            QRect min_text_rect(min_x, bar_row_y, current_min_text_width, bar_draw_height);
            QRect bar_rect(bar_x, bar_row_y, bar_draw_width, bar_draw_height);
            QRect max_text_rect(max_x, bar_row_y, current_max_text_width, bar_draw_height);
            QRect topic_text_rect(0, title_row_y, texture_image_.width(), current_text_height + 4); // Title spans full width, use padded height

            // Draw Min/Max Text
            painter.drawText(min_text_rect, Qt::AlignRight | Qt::AlignVCenter, min_str);
            painter.drawText(max_text_rect, Qt::AlignLeft | Qt::AlignVCenter, max_str);

            // Draw Bar Graph
            int available_width = bar_rect.width() - 2 * frame_thickness;
            int bar_fill_width = std::max(0, static_cast<int>(available_width * percentage));
            // Draw Frame
            painter.setPen(QPen(frame_color, frame_thickness));
            painter.setBrush(Qt::transparent);
            QRectF frame_draw_rect(bar_rect.left() + frame_thickness / 2.0, bar_rect.top() + frame_thickness / 2.0, bar_rect.width() - frame_thickness, bar_rect.height() - frame_thickness);
            painter.drawRect(frame_draw_rect);
            // Draw Fill
            QColor bar_color;
             if (last_msg_->current_color.a > 0.01) {
                 bar_color = QColor( static_cast<int>(last_msg_->current_color.r * 255.0), static_cast<int>(last_msg_->current_color.g * 255.0), static_cast<int>(last_msg_->current_color.b * 255.0), static_cast<int>(last_msg_->current_color.a * 255.0) );
             } else {
                 if (percentage < 0.5f) { bar_color = QColor::fromRgbF(1.0, percentage * 2.0, 0.0); } else { bar_color = QColor::fromRgbF(1.0 - (percentage - 0.5) * 2.0, 1.0, 0.0); }
             }
            if (bar_fill_width > 0) {
                painter.setPen(Qt::NoPen);
                painter.setBrush(bar_color);
                QRect bar_fill_rect(bar_rect.left() + frame_thickness, bar_rect.top() + frame_thickness, bar_fill_width, bar_rect.height() - 2 * frame_thickness);
                painter.drawRect(bar_fill_rect);
            }

            // Draw Current Value Text
            painter.setPen(text_color);
            painter.drawText(bar_rect, Qt::AlignCenter, current_str);

            // Draw Topic Text (Bottom row, centered)
            if (!title_text.isEmpty()) {
                painter.drawText(topic_text_rect.adjusted(0, 2, 0, -2), Qt::AlignCenter | Qt::AlignTop, title_text);
            }
        }
    }
  } // End if(draw_content)

  painter.end();

  // --- Upload Texture Data to Ogre ---
  try {
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
    // Lock the buffer for writing (discard previous contents)
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox & pixel_box = pixel_buffer->getCurrentLock();
    uint8_t * pDest = static_cast<uint8_t *>(pixel_box.data);

    // Check if Ogre's row pitch matches QImage's bytes per line
    size_t ogre_bytes_per_line = pixel_box.rowPitch * Ogre::PixelUtil::getNumElemBytes(pixel_box.format);
    size_t qimage_bytes_per_line = static_cast<size_t>(texture_image_.bytesPerLine());

    if (ogre_bytes_per_line == qimage_bytes_per_line) {
      // If they match, we can copy the entire buffer at once
      memcpy(pDest, texture_image_.constBits(), texture_image_.sizeInBytes());
    } else {
      // If they don't match (e.g., due to Ogre padding), copy row by row
      RVIZ_COMMON_LOG_DEBUG_STREAM("Ogre row pitch (" << ogre_bytes_per_line << ") differs from QImage bytes per line (" << qimage_bytes_per_line << "). Copying row by row.");
      size_t bytes_to_copy_per_line = std::min(ogre_bytes_per_line, qimage_bytes_per_line); // Copy the minimum to avoid buffer overflows
      for (int y = 0; y < texture_image_.height(); ++y) {
        memcpy(
          pDest + y * ogre_bytes_per_line, // Destination pointer advances by Ogre's pitch
          texture_image_.constScanLine(y), // Source pointer for the current row
          bytes_to_copy_per_line);         // Number of bytes in the row to copy
      }
    }
    // Unlock the buffer, making the changes visible to Ogre
    pixel_buffer->unlock();
  } catch (Ogre::Exception & e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error uploading texture data: " << e.getDescription());
  } catch (const std::exception& e) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Standard exception uploading texture data: " << e.what());
  } catch (...) {
      RVIZ_COMMON_LOG_ERROR("Unknown exception uploading texture data.");
  }
}

// Slot to update the service client when the property changes
void MinMaxCurrDisplay::updateCriticalService()
{
    std::string service_name = critical_service_name_property_->getStdString();
    critical_service_client_.reset(); // Reset existing client first

    if (service_name.empty()) {
        setStatus(rviz_common::properties::StatusProperty::Ok, "Critical Service", "No service configured.");
        RVIZ_COMMON_LOG_INFO("Critical service name cleared. Disabling service calls.");
        return;
    }

    try {
        // Get the node instance from the display context
        auto node_abstraction = context_->getRosNodeAbstraction().lock();
        if (!node_abstraction) {
            RVIZ_COMMON_LOG_ERROR("Could not get ROS node abstraction.");
            setStatus(rviz_common::properties::StatusProperty::Error, "Critical Service", "Failed to get node.");
            return;
        }
        auto node = node_abstraction->get_raw_node();
        if (!node) {
             RVIZ_COMMON_LOG_ERROR("Could not get raw ROS node pointer.");
             setStatus(rviz_common::properties::StatusProperty::Error, "Critical Service", "Failed to get node ptr.");
             return;
        }

        critical_service_client_ = node->create_client<TriggerCriticalAction>(service_name);
        setStatus(rviz_common::properties::StatusProperty::Ok, "Critical Service", "Client created for: " + QString::fromStdString(service_name));
        RVIZ_COMMON_LOG_INFO_STREAM("Created critical service client for: " << service_name);
    } catch (const std::exception& e) {
        RVIZ_COMMON_LOG_ERROR_STREAM("Failed to create critical service client: " << e.what());
        setStatus(rviz_common::properties::StatusProperty::Error, "Critical Service", "Client creation failed.");
    }
}

// Helper function to convert message to a simple JSON string
std::string MinMaxCurrDisplay::messageToJson(const min_max_curr_rviz_overlay::msg::MinMaxCurr& msg)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4); // Control float precision

    ss << "{";
    ss << "\"title\":\"" << msg.title << "\","; // Basic string escaping might be needed for complex titles
    ss << "\"min\":" << msg.min << ",";
    ss << "\"max\":" << msg.max << ",";
    ss << "\"current\":" << msg.current << ",";
    ss << "\"current_color\":{"
       << "\"r\":" << msg.current_color.r << ","
       << "\"g\":" << msg.current_color.g << ","
       << "\"b\":" << msg.current_color.b << ","
       << "\"a\":" << msg.current_color.a << "},";
    ss << "\"critical_value\":" << msg.critical_value << ",";
    ss << "\"critical_if_under\":" << (msg.critical_if_under ? "true" : "false") << ",";
    ss << "\"critical_animation_type\":" << static_cast<int>(msg.critical_animation_type) << ",";
    ss << "\"critical_color\":{"
       << "\"r\":" << msg.critical_color.r << ","
       << "\"g\":" << msg.critical_color.g << ","
       << "\"b\":" << msg.critical_color.b << ","
       << "\"a\":" << msg.critical_color.a << "},";
    ss << "\"precision\":" << static_cast<int>(msg.precision) << ",";
    ss << "\"compact\":" << (msg.compact ? "true" : "false") << ",";
    ss << "\"critical_service_name\":\"" << msg.critical_service_name << "\""; // Include service name from msg itself
    ss << "}";

    return ss.str();
}

} // namespace min_max_curr_rviz_overlay

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(min_max_curr_rviz_overlay::MinMaxCurrDisplay, rviz_common::Display)
