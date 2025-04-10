#ifndef WIFI_VIZ_BATTERY_STATE_DISPLAY_HPP
#define WIFI_VIZ_BATTERY_STATE_DISPLAY_HPP

#include <memory>
#include <string>
#include <rviz_common/display.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_rendering/objects/effort_visual.hpp>
#include "rviz_common/visibility_control.hpp"
#include "wifi_viz/battery_bar_visual.hpp"  // Include the new header

namespace wifi_viz
{

class RVIZ_COMMON_PUBLIC BatteryStateDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  BatteryStateDisplay();
  ~BatteryStateDisplay() override;

  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr overlayPublisher_;

  // Properties
  rviz_common::properties::StringProperty* topic_property_;
  rviz_common::properties::IntProperty* font_size_property_;
  rviz_common::properties::FloatProperty* color_property_;
  rviz_common::properties::IntProperty* x_position_property_;
  rviz_common::properties::IntProperty* y_position_property_;
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::IntProperty* screen_x_property_;
  rviz_common::properties::IntProperty* screen_y_property_;

  BatteryBarVisual* battery_bar_visual_;  // Add the BatteryBarVisual member

public Q_SLOTS:
  void updateSubscription();
};

}  // namespace wifi_viz

#endif  // WIFI_VIZ_BATTERY_STATE_DISPLAY_HPP