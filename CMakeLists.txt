cmake_minimum_required(VERSION 3.8)
project(min_max_curr_rviz_overlay)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
find_package(python_cmake_module REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Add message and service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MinMaxCurr.msg"
  "srv/TriggerCriticalAction.srv"
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
  include/min_max_curr_rviz_overlay/min_max_curr_display.hpp
)

# Force symbol visibility for exported classes
set(CMAKE_CXX_VISIBILITY_PRESET default)
set(CMAKE_VISIBILITY_INLINES_HIDDEN OFF)

# Include directories
include_directories(
  include
  ${rviz_ogre_vendor_INCLUDE_DIRS}
)

# Add the library with a distinct name
set(library_target ${PROJECT_NAME}_lib)
add_library(${library_target} SHARED
  src/min_max_curr_display.cpp
  ${MOC_FILES}
)

# Set visibility macros for the library target
target_compile_definitions(${library_target} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# Add dependencies for the library target
ament_target_dependencies(${library_target}
  rclcpp
  rviz_common
  rviz_ogre_vendor
  pluginlib
  rosidl_default_runtime
)

# Link Qt, Ogre, and the generated message typesupport library
target_link_libraries(${library_target}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  ${rviz_ogre_vendor_LIBRARIES}
  ${cpp_typesupport_target}
)

# Export include directory for the library target
target_include_directories(${library_target} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export the plugin
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install the library target
install(TARGETS ${library_target}
  EXPORT ${library_target}_export
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

# Install Python scripts using absolute path
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/default_critical_action_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Exports
ament_export_targets(${library_target}_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_ogre_vendor
  pluginlib
  Qt5
  rosidl_default_runtime
  std_msgs
  rclpy
)

# Export runtime interfaces
ament_export_dependencies(rosidl_default_runtime)

# Register package - MUST BE LAST
ament_package()
