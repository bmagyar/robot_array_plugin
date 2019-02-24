#ifndef RVIZ_ROBOT_ARRAY_DISPLAY_H_
#define RVIZ_ROBOT_ARRAY_DISPLAY_H_

#include <geometry_msgs/PoseArray.h>

#include <rviz/message_filter_display.h>
// #include <rviz/robot/robot.h>

namespace Ogre
{
class ManualObject;
};

namespace rviz
{
class ColorProperty;
class FloatProperty;
class EnumProperty;
class Axes;
class Robot;
class StringProperty;
}  // namespace rviz

namespace robot_array_plugin
{
/** @brief Displays a geometry_msgs/PoseArray message as a bunch of line-drawn arrows. */
class RobotArrayDisplay : public rviz::MessageFilterDisplay<geometry_msgs::PoseArray>
{
    Q_OBJECT
  public:
    enum Shape
    {
        Arrow,
        Axes,
    };
    RobotArrayDisplay();
    virtual ~RobotArrayDisplay();

    virtual void onInitialize();
    virtual void reset();

  private Q_SLOTS:
    void updateVisualVisible();
    void updateCollisionVisible();
    void updateTfPrefix();
    void updateAlpha();
    void updateRobotDescription();

  private:
    virtual void processMessage(const geometry_msgs::PoseArray::ConstPtr& msg);

    Ogre::ManualObject* manual_object_;

    rviz::ColorProperty* color_property_;
    std::vector<Ogre::SceneNode*> coords_nodes_;
    rviz::Property* visual_enabled_property_;
    rviz::Property* collision_enabled_property_;
    rviz::FloatProperty* update_rate_property_;
    rviz::StringProperty* robot_description_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::StringProperty* tf_prefix_property_;

    bool pose_valid_;

    /** @brief Loads a URDF from the ros-param named by our
    * "Robot Description" property, iterates through the links, and
    * loads any necessary models.
    */
    virtual void load();
    void clear();

    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    rviz::Robot* robot_;  ///< Handles actually drawing the robot

    bool has_new_transforms_;  ///< Callback sets this to tell our update function it needs to update the transforms

    float time_since_last_transform_;

    std::string robot_description_;
};

}  // namespace robot_array_plugin

#endif /* RVIZ_ROBOT_ARRAY_DISPLAY_H_ */
