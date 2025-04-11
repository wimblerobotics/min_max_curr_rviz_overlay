#include <Ogre.h>  // Add this line, move to the top
#include "wifi_viz/battery_bar_visual.hpp"
#include <rclcpp/rclcpp.hpp>  // Add this line
#include <OgreEntity.h>
#include <OgreHardwareBuffer.h>  // Add missing include
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>  // Add missing include
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>

namespace wifi_viz {
BatteryBarVisual::BatteryBarVisual(Ogre::SceneManager* scene_manager)
    : rviz_rendering::Object(scene_manager),
      bar_entity_(nullptr),
      frame_entity_(nullptr),
      min_voltage_(0.0f),
      max_voltage_(100.0f),
      width_(200),
      height_(20),
      position_(Ogre::Vector3::ZERO),
      orientation_(Ogre::Quaternion::IDENTITY) {
  static int count = 0;
  std::string name = "BatteryBarVisual" + std::to_string(count++);

  // Create the root scene node
  root_node_ = scene_manager_->createSceneNode(name + "RootNode");
  bar_node_ = scene_manager_->createSceneNode(name + "BarNode");
  frame_node_ = scene_manager_->createSceneNode(name + "FrameNode");
  root_node_->addChild(bar_node_);
  root_node_->addChild(frame_node_);

  // Attach root node to scene manager's root node
  scene_manager_->getRootSceneNode()->addChild(root_node_);

  // Create the bar mesh
  Ogre::MeshPtr bar_mesh = Ogre::MeshManager::getSingleton().createPlane(
      name + "BarMesh", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::Plane(Ogre::Vector3::UNIT_Z, 0), 1, 1, 1, 1, true,
      Ogre::HardwareBuffer::HBU_STATIC, Ogre::HardwareBuffer::HBU_STATIC);

  // Create the frame mesh (slightly larger than the bar)
  Ogre::MeshPtr frame_mesh = Ogre::MeshManager::getSingleton().createPlane(
      name + "FrameMesh",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::Plane(Ogre::Vector3::UNIT_Z, 0), 1, 1, 1, 1, true,
      Ogre::HardwareBuffer::HBU_STATIC, Ogre::HardwareBuffer::HBU_STATIC);

  // Create entities
  bar_entity_ = scene_manager_->createEntity(name + "BarEntity", bar_mesh);
  frame_entity_ =
      scene_manager_->createEntity(name + "FrameEntity", frame_mesh);
  bar_node_->attachObject(bar_entity_);
  frame_node_->attachObject(frame_entity_);

  // Create materials
  material_ = Ogre::MaterialManager::getSingleton().create(
      name + "Material",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setAmbient(0.0f, 1.0f, 0.0f);
  material_->setDiffuse(0.0f, 0.75f, 0.0f, 1.0f);
  material_->setSpecular(0.0f, 1.0f, 0.0f, 1.0f);
  material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material_->setDepthWriteEnabled(false);
  material_->setDepthCheckEnabled(false);

  frame_material_ = Ogre::MaterialManager::getSingleton().create(
      name + "FrameMaterial",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  frame_material_->setAmbient(1.0f, 1.0f, 1.0f);
  frame_material_->setDiffuse(1.0f, 1.0f, 1.0f, 1.0f);
  frame_material_->setSpecular(1.0f, 1.0f, 1.0f, 1.0f);
  frame_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  frame_material_->setDepthWriteEnabled(false);
  frame_material_->setDepthCheckEnabled(false);

  bar_entity_->setMaterial(material_);
  frame_entity_->setMaterial(frame_material_);

  // Set initial position and scale
  root_node_->setPosition(Ogre::Vector3(0, 0, 0));
  root_node_->setScale(Ogre::Vector3(1, 1, 1));

  // Set initial dimensions
  setDimensions(width_, height_);

  // Initialize bar width to zero
  bar_node_->setScale(0.0f, 1.0f, 1.0f);

  // Make sure the bar is visible
  bar_entity_->setVisible(true);
  frame_entity_->setVisible(true);
}

BatteryBarVisual::~BatteryBarVisual() {
  scene_manager_->destroySceneNode(root_node_);
  scene_manager_->destroyEntity(bar_entity_);
  scene_manager_->destroyEntity(frame_entity_);
  Ogre::MeshManager::getSingleton().remove(bar_entity_->getMesh()->getName());
  Ogre::MeshManager::getSingleton().remove(frame_entity_->getMesh()->getName());
  Ogre::MaterialManager::getSingleton().remove(material_->getName());
  Ogre::MaterialManager::getSingleton().remove(frame_material_->getName());
}

void BatteryBarVisual::setDimensions(int width, int height) {
  width_ = width;
  height_ = height;

  // Convert pixels to world units (assuming 100 pixels = 1 world unit)
  float pixel_scale = 0.01f;
  float world_width = width_ * pixel_scale;
  float world_height = height_ * pixel_scale;

  // Scale the frame to be the exact size as the bar
  frame_node_->setScale(world_width, world_height, 1.0f);
  RCLCPP_INFO(rclcpp::get_logger("setDimensions"),
              "frame node scale: %f, %f, %f", world_width,
              world_height, 1.0f);

  // Position the bar and frame
  bar_node_->setPosition(0, 0, 0.1f);  // Slightly in front of the frame
  frame_node_->setPosition(0, 0, 0);
}

void BatteryBarVisual::setScreenPosition(int x, int y) {
  // Convert screen position to world position
  float pixel_scale = 0.01f;
  Ogre::Vector3 world_position(x * pixel_scale, y * pixel_scale, 0.0f);
  setPosition(world_position);
}

void BatteryBarVisual::setVoltage(float voltage) {
  float clamped_voltage =
      std::max(min_voltage_, std::min(max_voltage_, voltage));
  float normalized_voltage =
      (clamped_voltage - min_voltage_) / (max_voltage_ - min_voltage_);

  // Convert pixels to world units
  float pixel_scale = 0.01f;
  float world_width = width_ * pixel_scale;
  float world_height = height_ * pixel_scale;

  // Calculate the width of the inner bar based on the normalized voltage
  float bar_width = normalized_voltage * width_;
  float world_bar_width = bar_width * pixel_scale;

  // Scale the bar horizontally based on voltage
  bar_node_->setScale(world_bar_width, world_height, 1.0f);

  // Position the bar so it grows from the left
  bar_node_->setPosition((-world_width / 2.0f) + (world_bar_width / 2.0f), 0.0f,
                         0.1f);

  // Update color based on voltage level
  float r = 1.0f - normalized_voltage;  // Red increases as voltage decreases
  float g = normalized_voltage;         // Green increases as voltage increases
  float b = 0.0f;
  setColor(r, g, b, 1.0f);
}

void BatteryBarVisual::setPosition(const Ogre::Vector3& position) {
  position_ = position;
  root_node_->setPosition(position);
}

void BatteryBarVisual::setScale(const Ogre::Vector3& scale) {
  root_node_->setScale(scale);
}

void BatteryBarVisual::setColor(float r, float g, float b, float a) {
  material_->setAmbient(r, g, b);
  material_->setDiffuse(r, g, b, a);
  material_->setSpecular(r, g, b, a);
}

Ogre::Vector3& BatteryBarVisual::getPosition() { return position_; }

Ogre::Quaternion& BatteryBarVisual::getOrientation() { return orientation_; }

void BatteryBarVisual::setUserData(const Ogre::Any& data) {
  bar_entity_->getUserObjectBindings().setUserAny(data);
}

void BatteryBarVisual::setOrientation(const Ogre::Quaternion& orientation) {
  // Keep the orientation fixed to screen space
  root_node_->setOrientation(Ogre::Quaternion::IDENTITY);
}
}  // namespace wifi_viz
