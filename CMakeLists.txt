cmake_minimum_required(VERSION 3.5)
project(wifi_viz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED) # Needed for Ogre overlay classes
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui) # Gui needed for QPainter/QImage

# Enable Qt moc/uic/rcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Make sure MOC finds all necessary headers
set(CMAKE_AUTOMOC_SEARCH_PATHS include src)

# Add explicit MOC processing for our display class
qt5_wrap_cpp(MOC_FILES
  include/wifi_viz/battery_state_display.hpp
)

# Force symbol visibility for exported classes
set(CMAKE_CXX_VISIBILITY_PRESET default)
set(CMAKE_VISIBILITY_INLINES_HIDDEN OFF)

# Include directories
include_directories(
  include
  ${rviz_ogre_vendor_INCLUDE_DIRS} # Add Ogre includes
)

# Add the library with the moc files
add_library(${PROJECT_NAME} SHARED
  src/battery_state_display.cpp
  ${MOC_FILES}
)

# Set visibility macros
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  rviz_common
  rviz_ogre_vendor # Add Ogre dependency
  pluginlib
)

# Link Qt and Ogre libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  ${rviz_ogre_vendor_LIBRARIES} # Link Ogre libraries
)

# Export include directory
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export the plugin
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
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

# Exports
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  rviz_common
  rviz_ogre_vendor # Add Ogre dependency
  pluginlib
  Qt5
)

ament_package()
