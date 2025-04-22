#ifndef WIFI_VIZ__WIFI_STATE_DISPLAY_HPP_
#define WIFI_VIZ__WIFI_STATE_DISPLAY_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "wifi_viz/msg/min_max_curr.hpp"

#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

// Ogre-specific headers
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

// Qt headers for drawing
#include <QColor>
#include <QImage>
#include <QFont>

// Forward declarations for Ogre classes
namespace Ogre
{
class Overlay;
class PanelOverlayElement;
} // namespace Ogre

namespace wifi_viz
{

// Ensure the template argument matches the message type
class WifiStateDisplay : public rviz_common::RosTopicDisplay<wifi_viz::msg::MinMaxCurr>
{
  Q_OBJECT

public:
  WifiStateDisplay();
  ~WifiStateDisplay() override;

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
  // Declare the helper function
  void calculateDimensions(
    int& total_width, int& total_height,
    int& bar_width, int& bar_height,
    int& min_text_w, int& max_text_w, int& topic_text_w, int& text_h);

  // RViz Properties
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::ColorProperty * frame_color_property_;
  rviz_common::properties::ColorProperty * text_color_property_;
  rviz_common::properties::IntProperty * font_size_property_;
  rviz_common::properties::BoolProperty * vertical_mode_property_;

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
  QImage texture_image_;
  bool needs_redraw_;
  int text_height_;
  int min_text_width_;
  int max_text_width_;
  int topic_text_width_; // Add member variable declaration
  int text_margin_;

  // State
  wifi_viz::msg::MinMaxCurr::ConstSharedPtr last_msg_;
  bool show_critical_flash_;
  std::chrono::time_point<std::chrono::steady_clock> last_flash_time_;
};

} // namespace wifi_viz

#endif // WIFI_VIZ__WIFI_STATE_DISPLAY_HPP_