#include "wifi_viz/battery_state_display.hpp"
#include "wifi_viz/battery_bar_visual.hpp"  // Include the battery bar visual
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rclcpp/rclcpp.hpp> // Include for RCLCPP_INFO

namespace wifi_viz
{

BatteryStateDisplay::BatteryStateDisplay()
  : rviz_common::Display()
  , battery_bar_visual_(nullptr)  // Initialize the battery bar visual
{
  // Create properties
  topic_property_ = new rviz_common::properties::StringProperty("Topic", "/battery_state",
                                                                 "Topic to subscribe to.", this,
                                                                 SLOT(updateSubscription()));

  font_size_property_ = new rviz_common::properties::IntProperty("Font Size", 12,
                                                                  "Font size of the overlay text.", this);
  font_size_property_->setMin(1);
  font_size_property_->setMax(100);

  color_property_ = new rviz_common::properties::FloatProperty("Color", 1.0,
                                                                "Color of the overlay text.", this);
  color_property_->setMin(0.0);
  color_property_->setMax(1.0);

  x_position_property_ = new rviz_common::properties::IntProperty("X Position", 10,
                                                                    "X position of the overlay.", this);
  y_position_property_ = new rviz_common::properties::IntProperty("Y Position", 10,
                                                                    "Y position of the overlay.", this);

  // Add width and height properties
  width_property_ = new rviz_common::properties::IntProperty("Width", 200,
                                                              "Width of the battery bar in pixels.", this);
  width_property_->setMin(1);
  width_property_->setMax(1000);
  
  height_property_ = new rviz_common::properties::IntProperty("Height", 20,
                                                               "Height of the battery bar in pixels.", this);
  height_property_->setMin(1);
  height_property_->setMax(100);

  // Add screen position properties
  screen_x_property_ = new rviz_common::properties::IntProperty("Screen X", 100,
                                                                 "X position on screen in pixels.", this);
  screen_x_property_->setMin(0);
  screen_x_property_->setMax(2000);
  
  screen_y_property_ = new rviz_common::properties::IntProperty("Screen Y", 50,
                                                                 "Y position on screen in pixels.", this);
  screen_y_property_->setMin(0);
  screen_y_property_->setMax(2000);
}

BatteryStateDisplay::~BatteryStateDisplay()
{
  delete battery_bar_visual_;
}

void BatteryStateDisplay::onInitialize()
{
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    throw std::runtime_error("Failed to lock ROS node abstraction");
  }

  auto node = ros_node_abstraction->get_raw_node();
  overlayPublisher_ = node->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("overlay_text", 10);

  battery_bar_visual_ = new BatteryBarVisual(context_->getSceneManager());  // Initialize the visual

  updateSubscription();
}

void BatteryStateDisplay::reset()
{
  subscription_.reset();
}

void BatteryStateDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;  // Suppress unused parameter warning
  (void)ros_dt;   // Suppress unused parameter warning
}

void BatteryStateDisplay::processMessage(sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "Received BatteryState message");
  RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "Voltage: %f", msg->voltage);

  RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "Font Size: %d", font_size_property_->getInt());
  RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "Color: %f", color_property_->getFloat());
  RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "X Position: %d", x_position_property_->getInt());
  RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "Y Position: %d", y_position_property_->getInt());

  battery_bar_visual_->setVoltage(msg->voltage);
  battery_bar_visual_->setDimensions(width_property_->getInt(), height_property_->getInt());
  
  // Update screen position
  battery_bar_visual_->setScreenPosition(screen_x_property_->getInt(), screen_y_property_->getInt());

  rviz_2d_overlay_msgs::msg::OverlayText overlay;
  overlay.text = "Voltage: " + std::to_string(msg->voltage) + " V";
  overlay.text_size = font_size_property_->getInt();
  overlay.bg_color.r = color_property_->getFloat();
  overlay.bg_color.g = color_property_->getFloat();
  overlay.bg_color.b = color_property_->getFloat();
  overlay.bg_color.a = 1.0;
  overlay.horizontal_distance = x_position_property_->getInt();
  overlay.vertical_distance = y_position_property_->getInt();
  overlayPublisher_->publish(overlay);
}

void BatteryStateDisplay::updateSubscription()
{
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    throw std::runtime_error("Failed to lock ROS node abstraction");
  }

  auto node = ros_node_abstraction->get_raw_node();
  const std::string& topic_name = topic_property_->getStdString();

  if (topic_name.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "Topic name is empty, not subscribing");
    return;
  }

  subscription_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    topic_name, 10,
    [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) { processMessage(msg); });

  RCLCPP_INFO(rclcpp::get_logger("battery_state_display"), "Subscribed to topic: %s", topic_name.c_str());
}

}  // namespace wifi_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wifi_viz::BatteryStateDisplay, rviz_common::Display)