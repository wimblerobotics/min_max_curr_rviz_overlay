#ifndef WIFI_VIZ_BATTERY_STATE_DISPLAY_HPP_
#define WIFI_VIZ_BATTERY_STATE_DISPLAY_HPP_

#include <memory>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "wifi_viz/battery_bar_visual.hpp"

namespace wifi_viz
{
class BatteryStateDisplay : public rviz_common::RosTopicDisplay<sensor_msgs::msg::BatteryState>
{
  Q_OBJECT

public:
  BatteryStateDisplay();
  ~BatteryStateDisplay() override;

protected:
  void onEnable() override;
  void onDisable() override;
  void onInitialize() override;
  void reset() override;
  void processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateVisual();

private:
  std::unique_ptr<BatteryBarVisual> battery_bar_visual_;
  rviz_common::properties::FloatProperty* min_voltage_property_;
  rviz_common::properties::FloatProperty* max_voltage_property_;
  rviz_common::properties::IntProperty* width_property_;
  rviz_common::properties::IntProperty* height_property_;
  rviz_common::properties::IntProperty* screen_x_property_;
  rviz_common::properties::IntProperty* screen_y_property_;
};

}  // namespace wifi_viz

#endif  // WIFI_VIZ_BATTERY_STATE_DISPLAY_HPP_