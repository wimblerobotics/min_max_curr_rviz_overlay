cmake_minimum_required(VERSION 3.8)
project(wifi_viz)

# Force building shared libraries only
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# Add -fPIC and -rdynamic flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -rdynamic")

# Add debug flags
set(CMAKE_CXX_FLAGS_DEBUG "-g")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rviz_default_plugins REQUIRED)

set(rviz_default_plugins_headers_to_mock
  include/wifi_viz/battery_state_display.hpp
)
foreach(header "${rviz_default_plugins_headers_to_mock}")
  qt5_wrap_cpp(rviz_default_plugins_moc_files "${header}")
endforeach()

set(rviz_default_plugins_source_files
  src/battery_state_display.cpp
  src/battery_bar_visual.cpp # Add the new source file
)

# Include directories
include_directories(
  include
  ${rviz_rendering_INCLUDE_DIRS} # add the rviz_rendering include directory
)

# Add library
add_library(${PROJECT_NAME} SHARED
  ${rviz_default_plugins_moc_files}
  ${rviz_default_plugins_source_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${rviz_default_plugins}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_2d_overlay_msgs::rviz_2d_overlay_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  rclcpp::rclcpp
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  resource_retriever::resource_retriever
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  resource_retriever::resource_retriever
)

# Ensure symbols are exported
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# # Install plugin description to the correct location for RViz2
# install(FILES plugin_description.xml
#   DESTINATION share/rviz2/plugins
# )

# Export plugin description file
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package(
  # CONFIG_EXTRAS "rviz_default_plugins-extras.cmake"
)
