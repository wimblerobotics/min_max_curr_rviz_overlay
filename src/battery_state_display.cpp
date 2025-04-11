#include "wifi_viz/battery_state_display.hpp"
#include "wifi_viz/battery_bar_visual.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wifi_viz
{

BatteryStateDisplay::BatteryStateDisplay()
  : min_voltage_property_(nullptr)
  , max_voltage_property_(nullptr)
  , width_property_(nullptr)
  , height_property_(nullptr)
  , screen_x_property_(nullptr)
  , screen_y_property_(nullptr)
{
  // Add properties for voltage range
  min_voltage_property_ = new rviz_common::properties::FloatProperty(
    "Min Voltage", 0.0f,
    "Minimum voltage value for scaling",
    this, SLOT(updateVisual()));
  min_voltage_property_->setMin(0.0f);

  max_voltage_property_ = new rviz_common::properties::FloatProperty(
    "Max Voltage", 100.0f,
    "Maximum voltage value for scaling",
    this, SLOT(updateVisual()));
  max_voltage_property_->setMin(0.0f);

  // Add properties for dimensions
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 200,
    "Width of the battery bar in pixels",
    this, SLOT(updateVisual()));
  width_property_->setMin(1);
  width_property_->setMax(2000);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 20,
    "Height of the battery bar in pixels",
    this, SLOT(updateVisual()));
  height_property_->setMin(1);
  height_property_->setMax(200);

  // Add properties for screen position
  screen_x_property_ = new rviz_common::properties::IntProperty(
    "Screen X", 100,
    "X position on screen in pixels",
    this, SLOT(updateVisual()));
  screen_x_property_->setMin(0);
  screen_x_property_->setMax(2000);

  screen_y_property_ = new rviz_common::properties::IntProperty(
    "Screen Y", 50,
    "Y position on screen in pixels",
    this, SLOT(updateVisual()));
  screen_y_property_->setMin(0);
  screen_y_property_->setMax(2000);
}

BatteryStateDisplay::~BatteryStateDisplay()
{
  // Battery bar visual is automatically cleaned up by unique_ptr
}

void BatteryStateDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  
  // Create the visual
  battery_bar_visual_ = std::make_unique<BatteryBarVisual>(scene_manager_);
  
  // Initialize the visual with current properties
  updateVisual();
}

void BatteryStateDisplay::onEnable()
{
  RosTopicDisplay::onEnable();
  
  // Make sure the visual exists
  if (!battery_bar_visual_) {
    battery_bar_visual_ = std::make_unique<BatteryBarVisual>(scene_manager_);
  }
  
  // Update visual state
  updateVisual();
}

void BatteryStateDisplay::onDisable()
{
  RosTopicDisplay::onDisable();
}

void BatteryStateDisplay::reset()
{
  RosTopicDisplay::reset();
}

void BatteryStateDisplay::processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  if (!battery_bar_visual_ || !isEnabled()) {
    return;
  }

  // Log the received message
  RCLCPP_INFO(rclcpp::get_logger("BatteryStateDisplay"), 
              "Received battery voltage: %.2f", msg->voltage);

  // Update the visual with the new voltage
  battery_bar_visual_->setVoltage(msg->voltage);
  
  // Make sure the visual is still properly positioned
  updateVisual();
}

void BatteryStateDisplay::updateVisual()
{
  if (!battery_bar_visual_ || !isEnabled()) {
    return;
  }

  // Set the voltage range on the visual
  // battery_bar_visual_->setVoltageRange(min_voltage_property_->getFloat(), max_voltage_property_->getFloat());
  
  // Update dimensions and position
  battery_bar_visual_->setDimensions(width_property_->getInt(), height_property_->getInt());
  battery_bar_visual_->setScreenPosition(screen_x_property_->getInt(), screen_y_property_->getInt());
  
  // Request a render update
  context_->queueRender();
}

}  // namespace wifi_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wifi_viz::BatteryStateDisplay, rviz_common::Display)