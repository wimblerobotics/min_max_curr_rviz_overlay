#ifndef WIFI_VIZ__WIFI_STATE_DISPLAY_HPP_ // Renamed include guard
#define WIFI_VIZ__WIFI_STATE_DISPLAY_HPP_

#include <memory>
#include <string>

#include "wifi_viz/msg/min_max_curr.hpp"

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>

// Ogre-specific headers
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

// Qt headers for drawing
#include <QColor>
#include <QImage>
#include <QFont> // <<< Add QFont include

// Forward declarations for Ogre classes
namespace Ogre
{
class Overlay;
class PanelOverlayElement;
} // namespace Ogre

namespace wifi_viz
{

// Ensure the template argument matches the message type
class WifiStateDisplay : public rviz_common::RosTopicDisplay<wifi_viz::msg::MinMaxCurr> // Renamed class
{
  Q_OBJECT

public:
  WifiStateDisplay(); // Renamed constructor
  ~WifiStateDisplay() override; // Renamed destructor

  // RViz Display overrides
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  // Message handling overrides
  void processMessage(wifi_viz::msg::MinMaxCurr::ConstSharedPtr msg) override;

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
  rviz_common::properties::ColorProperty * text_color_property_; // <<< Add text color property
  rviz_common::properties::IntProperty * font_size_property_;   // <<< Add font size property

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
  int text_height_;      // <<< Add variable to store calculated text height
  int min_text_width_;   // <<< Add variable for min text width
  int max_text_width_;   // <<< Add variable for max text width
  int text_margin_;      // <<< Add variable for margin

  // State
  float current_value_;
  float min_value_;
  float max_value_;
};

} // namespace wifi_viz

#endif // WIFI_VIZ__WIFI_STATE_DISPLAY_HPP_ // Renamed include guard