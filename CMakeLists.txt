cmake_minimum_required(VERSION 3.5)
project(wifi_viz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED) # Needed for Ogre overlay classes
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui) # Gui needed for QPainter/QImage
find_package(rosidl_default_generators REQUIRED) # Added for message generation
find_package(std_msgs REQUIRED) # Add find_package for std_msgs

# Generate C++ code from the .msg file
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MinMaxCurr.msg"
  DEPENDENCIES std_msgs
)

# Get the C++ typesupport target name
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Enable Qt moc/uic/rcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Make sure MOC finds all necessary headers
set(CMAKE_AUTOMOC_SEARCH_PATHS include src)

# Add explicit MOC processing for our display class
qt5_wrap_cpp(MOC_FILES
  include/wifi_viz/wifi_state_display.hpp
)

# Force symbol visibility for exported classes
set(CMAKE_CXX_VISIBILITY_PRESET default)
set(CMAKE_VISIBILITY_INLINES_HIDDEN OFF)

# Include directories
include_directories(
  include
  ${rviz_ogre_vendor_INCLUDE_DIRS} # Add Ogre includes
)

# Add the library with a distinct name
set(library_target ${PROJECT_NAME}_lib) # Define a new variable for the library target name
add_library(${library_target} SHARED # Use the new variable here
  src/wifi_state_display.cpp
  # src/wifi_bar_visual.cpp # Uncomment if using this file
  ${MOC_FILES}
)

# Set visibility macros for the library target
target_compile_definitions(${library_target} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY") # Use ${library_target}

# Add dependencies for the library target
ament_target_dependencies(${library_target} # Use ${library_target}
  rclcpp
  rviz_common
  rviz_ogre_vendor
  pluginlib
  rosidl_default_runtime
)

# Link Qt, Ogre, and the generated message typesupport library
target_link_libraries(${library_target} # Use ${library_target}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  ${rviz_ogre_vendor_LIBRARIES}
  ${cpp_typesupport_target}
)

# Export include directory for the library target
target_include_directories(${library_target} PUBLIC # Use ${library_target}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export the plugin
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install the library target
install(TARGETS ${library_target} # Use ${library_target}
  EXPORT ${library_target}_export # Create export name based on library target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install the plugin description file
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/battery_to_minmax_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Exports
# Export ONLY the library target's export set
ament_export_targets(${library_target}_export HAS_LIBRARY_TARGET) # Use the export name matching install()
# Export dependencies
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_ogre_vendor
  pluginlib
  Qt5
  rosidl_default_runtime
  std_msgs
)

ament_package()
