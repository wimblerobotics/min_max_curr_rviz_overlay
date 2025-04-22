# Min/Max/Current Value RViz Overlay

This package provides an RViz display plugin that visualizes a numerical value relative to its minimum and maximum bounds, often presented as a horizontal or vertical bar. It includes features to indicate critical states through color changes or flashing animations.

## Features

*   Displays a value bar with current, min, and max labels.
*   Configurable position, size, colors, and font size via RViz properties panel.
*   Optional title override via the message.
*   Optional custom color for the bar fill via the message.
*   Optional critical state indication:
    *   Define a threshold (`critical_value`).
    *   Specify whether being *under* or *over* the threshold is critical (`critical_if_under`).
    *   Choose an animation type (`critical_animation_type`):
        *   `ANIMATION_NONE`: No visual change.
        *   `ANIMATION_COLORIZE`: Fill the display background with `critical_color`.
        *   `ANIMATION_FLASH`: Flash the display background with `critical_color`.
*   Configurable numeric precision for displayed values (`precision`).
*   Supports both horizontal and vertical bar orientation.
*   Optional compact layout (`compact`) to arrange elements differently (e.g., title beside bar).
*   Optional ROS 2 Service call on critical condition via RViz property (`Critical Service Name`).

## Installation

1.  **Clone the Repository:**
    Clone this repository into the `src` folder of your ROS 2 workspace (e.g., `~/min_max_curr_rviz_overlay_ws/src`).

    ```bash
    # cd ~/your_ros2_ws/src
    # git clone git@github.com:wimblerobotics/min_max_curr_rviz_overlay.git
    ```

2.  **Build the Package:**
    Navigate to the root of your workspace and build the package using `colcon`.

    ```bash
    cd ~/min_max_curr_rviz_overlay_ws
    colcon build --symlink-install --packages-select wifi_viz
    ```

3.  **Source the Overlay:**
    **Crucially**, before running RViz, you need to source the setup file of your workspace in the *same terminal* where you will launch RViz. This allows RViz to find the newly built plugin.

    ```bash
    cd ~/min_max_curr_rviz_overlay_ws
    source install/setup.bash
    ```
    *Note: You need to do this sourcing step in every new terminal you open before running `rviz2` if you want to use this overlay.*

## Usage

1.  **Launch RViz:**
    Make sure you have sourced your workspace as described above.

    ```bash
    rviz2
    ```

2.  **Add the Display:**
    *   In RViz, click the "Add" button in the "Displays" panel.
    *   Find the "MinMaxCurrDisplay" plugin under the "wifi_viz" category.
    *   Click "OK".

3.  **Configure the Display:**
    *   Select the "MinMaxCurrDisplay" in the Displays panel.
    *   Set the "Topic" property to the ROS 2 topic where `wifi_viz/msg/MinMaxCurr` messages will be published (e.g., `/battery_percentage_overlay`).
    *   **Critical Service Name**: Optionally, set this property to the name of a ROS 2 service (type `wifi_viz/srv/TriggerCriticalAction`) that should be called when the value enters a critical state (e.g., `/trigger_critical_action`). Leave empty to disable.
    *   Adjust other properties like width, height, position, colors, font size, and orientation as needed.

4.  **Run the Critical Action Service (Optional):**
    If you configured a "Critical Service Name", you need to run a node providing that service. A default logging service is included:
    ```bash
    # In a sourced terminal
    ros2 run wifi_viz default_critical_action_service.py
    ```

5.  **Publish Data:**
    Publish messages of type `wifi_viz/msg/MinMaxCurr` to the topic you configured.

## Examples

Here are some examples using `ros2 topic pub` to send a single message to the `/battery_percentage_overlay` topic. These assume the display in RViz is configured to listen to this topic.

**Example 1: Critical (Under Threshold), No Animation, Red Current Color**

Value (0.15) is below critical (0.25), but animation is NONE. The bar color will be red.

~![Example 1](media/overlay_example1.png)

```bash
ros2 topic pub --once /battery_percentage_overlay wifi_viz/msg/MinMaxCurr '{
  min: 0.0,
  max: 1.0,
  current: 0.15,
  critical_value: 0.25,
  critical_if_under: true,
  critical_animation_type: 0, # ANIMATION_NONE
  critical_python_function: "",
  title: "Battery %",
  compact: false,
  current_color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}, # Red
  critical_color: {r: 0.5, g: 0.0, b: 0.5, a: 1.0}  # Purple (Not used for background due to ANIMATION_NONE)
}'
```

**Example 2: Critical (Under Threshold), Colorize Animation, Purple Critical Color, Green Current Color, Precision 2**

Value (0.15) is below critical (0.25). Animation is COLORIZE, so the background will turn purple. The bar itself will be green. Values will be displayed with 2 decimal places.

~![Example 2](media/overlay_example2.png)

```bash
ros2 topic pub --once /battery_percentage_overlay wifi_viz/msg/MinMaxCurr '{
  min: 0.0,
  max: 1.0,
  current: 0.15,
  critical_value: 0.25,
  critical_if_under: true,
  critical_animation_type: 1, # ANIMATION_COLORIZE
  critical_python_function: "",
  title: "Battery %",
  compact: false,
  current_color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}, # Green
  critical_color: {r: 0.5, g: 0.0, b: 0.5, a: 1.0},  # Purple
  precision: 2
}'
```

**Example 3: Compact Layout, Critical (Under Threshold), Colorize Animation, Purple Critical Color, Green Current Color, Precision 2**

Same as Example 2, but with `compact: true`. In horizontal mode, the title moves to the left of the min value.

~![Example 3](media/overlay_compact.png)

```bash
ros2 topic pub --once /battery_percentage_overlay wifi_viz/msg/MinMaxCurr '{
  min: 0.0,
  max: 1.0,
  current: 0.15,
  critical_value: 0.25,
  critical_if_under: true,
  critical_animation_type: 1, # ANIMATION_COLORIZE
  critical_python_function: "",
  title: "Battery %",
  compact: true, # Enable compact layout
  current_color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}, # Green
  critical_color: {r: 0.5, g: 0.0, b: 0.5, a: 1.0},  # Purple
  precision: 2
}'
```

**Example 4: Critical (Under Threshold), Flash Animation, Blue Critical Color, Yellow Current Color**

Value (0.15) is below critical (0.25). Animation is FLASH, so the background will flash using the blue critical color. The bar itself will be yellow.

~![Example 4](media/overlay_example3.png)

```bash
ros2 topic pub --once /battery_percentage_overlay wifi_viz/msg/MinMaxCurr '{
  min: 0.0,
  max: 1.0,
  current: 0.15,
  critical_value: 0.25,
  critical_if_under: true,
  critical_animation_type: 2, # ANIMATION_FLASH
  critical_python_function: "",
  title: "Battery %",
  compact: false,
  current_color: {r: 1.0, g: 1.0, b: 0.0, a: 1.0}, # Yellow
  critical_color: {r: 0.0, g: 0.0, b: 1.0, a: 1.0}  # Blue
}'
```

**Example 5: Triggering the Critical Service**

This example assumes the display's "Critical Service Name" property is set to `/trigger_critical_action` and the `default_critical_action_service.py` node is running. When the message is published, the value (0.15) is critical, and the service will be called, logging the message data to the service node's console. The display itself will use COLORIZE animation.

```bash
ros2 topic pub --once /battery_percentage_overlay wifi_viz/msg/MinMaxCurr '{
  min: 0.0,
  max: 1.0,
  current: 0.15,
  critical_value: 0.25,
  critical_if_under: true,
  critical_animation_type: 1, # ANIMATION_COLORIZE
  critical_service_name: "/trigger_critical_action", # This field in msg is currently informational
  title: "Battery %",
  compact: false,
  current_color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}, # Green
  critical_color: {r: 0.5, g: 0.0, b: 0.5, a: 1.0},  # Purple
  precision: 2
}'
```

## Message Fields (`wifi_viz/msg/MinMaxCurr`)

*   `min` (float32): The minimum possible value for the range.
*   `max` (float32): The maximum possible value for the range.
*   `current` (float32): The current value to display.
*   `critical_value` (float32): The threshold for the critical state.
*   `critical_if_under` (bool): If true, the state is critical when `current < critical_value`. If false, critical when `current > critical_value`.
*   `critical_animation_type` (uint8): Type of animation for critical state (0: None, 1: Colorize Background, 2: Flash Background).
*   `critical_service_name` (string): Name of a ROS 2 service (type `wifi_viz/srv/TriggerCriticalAction`) intended to be called when a critical condition is met. (Note: This C++ plugin currently uses the RViz property "Critical Service Name" to configure the service call, not this message field directly).
*   `critical_python_function` (string): (Not currently used by this C++ display) Path to a Python function for custom critical logic.
*   `title` (string): Optional title displayed. Position depends on `compact` flag and orientation. If empty, the topic name is used.
*   `compact` (bool): If true, uses a more compact layout (e.g., title beside bar in horizontal mode). If false, uses the default layout (e.g., title below bar in horizontal mode).
*   `current_color` (std_msgs/ColorRGBA): Color used to draw the fill of the bar representing the current value. Alpha > 0 is required to override the default green/yellow/red gradient.
*   `critical_color` (std_msgs/ColorRGBA): Color used for the background during COLORIZE or FLASH animations when in a critical state.
*   `precision` (uint8): Number of decimal places to display for the min, max, and current values (default: 1).

## Service Definition (`wifi_viz/srv/TriggerCriticalAction`)

*   **Request:**
    *   `json_data` (string): The full `MinMaxCurr` message data that triggered the critical state, represented as a JSON string.
*   **Response:**
    *   `success` (bool): Indicates if the service logic executed successfully.
    *   `message` (string): An optional status message from the service.