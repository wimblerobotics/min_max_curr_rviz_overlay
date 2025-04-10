#include <Ogre.h> // Add this line, move to the top
#include "wifi_viz/battery_bar_visual.hpp"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreSubEntity.h>
#include <OgreMeshManager.h> // Add missing include
#include <OgreHardwareBuffer.h> // Add missing include

namespace wifi_viz
{
BatteryBarVisual::BatteryBarVisual(Ogre::SceneManager* scene_manager)
  : rviz_rendering::Object(scene_manager) // Corrected constructor
  , bar_entity_(nullptr)
  , min_voltage_(0.0f)
  , max_voltage_(100.0f)
  , position_(Ogre::Vector3::ZERO)
  , orientation_(Ogre::Quaternion::IDENTITY)
{
  static int count = 0;
  std::string name = "BatteryBarVisual" + std::to_string(count++);

  // Create the root scene node
  root_node_ = scene_manager_->createSceneNode(name + "RootNode");
  bar_node_ = scene_manager_->createSceneNode(name + "Node");
  root_node_->addChild(bar_node_);

  // Attach root node to scene manager's root node
  scene_manager_->getRootSceneNode()->addChild(root_node_);

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createPlane(
    name + "Mesh", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::Plane(Ogre::Vector3::UNIT_Z, 0), 1, 1, 1, 1, true, Ogre::HardwareBuffer::HBU_STATIC,
    Ogre::HardwareBuffer::HBU_STATIC);

  bar_entity_ = scene_manager_->createEntity(name + "Entity", mesh);
  bar_node_->attachObject(bar_entity_);

  material_ = Ogre::MaterialManager::getSingleton().create(
    name + "Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setAmbient(0.0f, 1.0f, 0.0f);
  material_->setDiffuse(0.0f, 0.75f, 0.0f, 1.0f);
  material_->setSpecular(0.0f, 1.0f, 0.0f, 1.0f);
  bar_entity_->setMaterial(material_);

  // Set initial position and scale
  root_node_->setPosition(Ogre::Vector3(0, 0, 0));
  root_node_->setScale(Ogre::Vector3(1, 1, 1));
}

BatteryBarVisual::~BatteryBarVisual()
{
  scene_manager_->destroySceneNode(root_node_);
  scene_manager_->destroyEntity(bar_entity_);
  Ogre::MeshManager::getSingleton().remove(bar_entity_->getMesh()->getName());
  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

void BatteryBarVisual::setVoltage(float voltage)
{
  float clamped_voltage = std::max(min_voltage_, std::min(max_voltage_, voltage));
  float normalized_voltage = (clamped_voltage - min_voltage_) / (max_voltage_ - min_voltage_);
  
  // Scale the bar vertically based on voltage
  bar_node_->setScale(1.0f, normalized_voltage, 1.0f);
  
  // Position the bar so it grows from the bottom
  bar_node_->setPosition(0.0f, -0.5f + (normalized_voltage * 0.5f), 0.0f);
  
  // Update color based on voltage level
  float r = 1.0f - normalized_voltage;  // Red increases as voltage decreases
  float g = normalized_voltage;         // Green increases as voltage increases
  float b = 0.0f;
  setColor(r, g, b, 1.0f);
}

void BatteryBarVisual::setPosition(const Ogre::Vector3& position)
{
  position_ = position;
  root_node_->setPosition(position);
}

void BatteryBarVisual::setOrientation(const Ogre::Quaternion& orientation)
{
  orientation_ = orientation;
  root_node_->setOrientation(orientation);
}

void BatteryBarVisual::setScale(const Ogre::Vector3& scale)
{
  root_node_->setScale(scale);
}

void BatteryBarVisual::setColor(float r, float g, float b, float a)
{
  material_->setAmbient(r, g, b);
  material_->setDiffuse(r, g, b, a);
}

Ogre::Vector3& BatteryBarVisual::getPosition()
{
  return position_;
}

Ogre::Quaternion& BatteryBarVisual::getOrientation()
{
  return orientation_;
}

void BatteryBarVisual::setUserData(const Ogre::Any& data)
{
  bar_entity_->getUserObjectBindings().setUserAny(data);
}
}  // namespace wifi_viz
