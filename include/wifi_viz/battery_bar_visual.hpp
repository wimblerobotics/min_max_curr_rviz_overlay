#ifndef WIFI_VIZ_BATTERY_BAR_VISUAL_HPP
#define WIFI_VIZ_BATTERY_BAR_VISUAL_HPP

#include <memory>
#include <string>
#include <rviz_rendering/objects/object.hpp> // Corrected include
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>

namespace wifi_viz
{
class BatteryBarVisual : public rviz_rendering::Object
{
public:
  BatteryBarVisual(Ogre::SceneManager* scene_manager);
  ~BatteryBarVisual() override;

  void setVoltage(float voltage);

  // Implement the pure virtual functions
  void setPosition(const Ogre::Vector3& position) override;
  void setOrientation(const Ogre::Quaternion& orientation) override;
  void setScale(const Ogre::Vector3& scale) override;
  void setColor(float r, float g, float b, float a) override;
  Ogre::Vector3& getPosition() override;
  Ogre::Quaternion& getOrientation() override;
  void setUserData(const Ogre::Any& data) override;

private:
  Ogre::SceneNode* root_node_;
  Ogre::SceneNode* bar_node_;
  Ogre::Entity* bar_entity_;
  Ogre::MaterialPtr material_;
  float min_voltage_;
  float max_voltage_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
};
}  // namespace wifi_viz

#endif  // WIFI_VIZ_BATTERY_BAR_VISUAL_HPP
