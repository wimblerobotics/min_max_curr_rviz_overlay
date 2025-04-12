#ifndef WIFI_VIZ__BATTERY_STATE_DISPLAY_HPP_
#define WIFI_VIZ__BATTERY_STATE_DISPLAY_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/battery_state.hpp>

#include <rviz_common/ros_topic_display.hpp> // Change base class include
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>

// Ogre-specific headers
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

// Qt headers for drawing
#include <QColor>
#include <QImage>

// Forward declarations for Ogre classes
namespace Ogre
{
class Overlay;
class PanelOverlayElement;
} // namespace Ogre

namespace wifi_viz
{

class BatteryStateDisplay : public rviz_common::RosTopicDisplay<sensor_msgs::msg::BatteryState>
{
  Q_OBJECT

public:
  BatteryStateDisplay();
  ~BatteryStateDisplay() override;

  // RViz Display overrides
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  // Message handling overrides
  void processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg) override;

private Q_SLOTS:
  // Update display based on property changes
  void updateProperties();
  // Update the texture drawn on the overlay
  void updateOverlayTexture();

private:
  // Helper to create/recreate Ogre texture
  void createTexture();
  // Helper to ensure Ogre overlay elements exist
  void ensureOverlay();

  // RViz Properties
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * left_property_; // Use 'left' instead of 'x'
  rviz_common::properties::IntProperty * top_property_;  // Use 'top' instead of 'y'
  rviz_common::properties::ColorProperty * frame_color_property_;

  // Ogre Overlay related members
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  std::string texture_name_;
  std::string material_name_;
  std::string overlay_name_;
  std::string panel_name_;

  // Drawing members
  QImage texture_image_; // QImage used as a canvas
  bool needs_redraw_;    // Flag to trigger texture update

  // State
  float current_voltage_;
  const float min_voltage_ = 0.0f;  // Fixed range
  const float max_voltage_ = 100.0f; // Fixed range
};

} // namespace wifi_viz

#endif // WIFI_VIZ__BATTERY_STATE_DISPLAY_HPP_