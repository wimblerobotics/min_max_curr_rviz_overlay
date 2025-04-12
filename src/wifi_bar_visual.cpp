#include <Ogre.h>
#include "wifi_viz/wifi_bar_visual.hpp" // Renamed include
#include <rclcpp/rclcpp.hpp>
// ... other includes ...

namespace wifi_viz {
// Renamed constructor
WifiBarVisual::WifiBarVisual(Ogre::SceneManager* scene_manager)
    : rviz_rendering::Object(scene_manager),
      bar_entity_(nullptr),
      frame_entity_(nullptr),
      min_value_(0.0f), // Renamed from min_voltage_
      max_value_(100.0f), // Renamed from max_voltage_
      width_(200),
      height_(20),
      position_(Ogre::Vector3::ZERO),
      orientation_(Ogre::Quaternion::IDENTITY) {
  static int count = 0;
  std::string name = "WifiBarVisual" + std::to_string(count++); // Renamed prefix

  // ... rest of constructor ...
}

// Renamed destructor
WifiBarVisual::~WifiBarVisual() {
  // ... existing destructor code ...
}

// Renamed setDimensions
void WifiBarVisual::setDimensions(int width, int height) {
  // ... existing setDimensions code ...
  RCLCPP_INFO(rclcpp::get_logger("setDimensions"), // Logger name unchanged, but context is wifi
              "frame size: %f x %f, bar_container at x=%f",
              world_width, world_height, frame_left_edge);
}

// Renamed setScreenPosition
void WifiBarVisual::setScreenPosition(int x, int y) {
  // ... existing setScreenPosition code ...
}

// Renamed setValue (from setVoltage)
void WifiBarVisual::setValue(float value) { // Renamed parameter
  float clamped_value = // Renamed from clamped_voltage
      std::max(min_value_, std::min(max_value_, value)); // Use renamed members
  float normalized_value = // Renamed from normalized_voltage
      (clamped_value - min_value_) / (max_value_ - min_value_); // Use renamed members

  // ... existing code ...

  // Calculate the width of the inner bar based on the normalized value
  float bar_width = normalized_value * width_; // Use renamed variable
  // ... existing code ...

  RCLCPP_INFO(rclcpp::get_logger("setValue"), // Logger name unchanged, but context is wifi
              "Value: %.2f (%.2f%%), bar width: %.2f, half_width: %.2f", // Updated log text
              value, normalized_value * 100.0f, world_bar_width, half_bar_width); // Use renamed variables

  // Update color based on value level
  float r = 1.0f - normalized_value; // Use renamed variable
  float g = normalized_value;        // Use renamed variable
  // ... existing code ...
}

// ... setPosition, setScale, setColor, getPosition, getOrientation, setUserData, setOrientation ...

}  // namespace wifi_viz
