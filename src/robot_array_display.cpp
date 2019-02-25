#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <tinyxml.h>
#include <urdf/model.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/validate_floats.h>

#include "robot_array_display.h"

using rviz::FloatProperty;
using rviz::Property;
using rviz::Robot;
using rviz::StringProperty;
using rviz::ColorProperty;
using rviz::StatusProperty;
using rviz::TFLinkUpdater;

namespace robot_array_plugin
{
inline void linkUpdaterStatusFunction(StatusProperty::Level level, const std::string& link_name,
                                      const std::string& text, RobotArrayDisplay* display)
{
    display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
}

inline Ogre::Vector3 toOgre(const geometry_msgs::Point& position)
{
    Ogre::Vector3 result;
    result.x = position.x;
    result.y = position.y;
    result.z = position.z;
    return result;
}

inline Ogre::Quaternion toOgre(const geometry_msgs::Quaternion& orientation)
{
    Ogre::Quaternion result;
    result.x = orientation.x;
    result.y = orientation.y;
    result.z = orientation.z;
    result.w = orientation.w;
    return result;
}

RobotArrayDisplay::RobotArrayDisplay() : manual_object_(NULL)
{
    color_property_ = new ColorProperty("Color", QColor(255, 25, 0), "Color to draw the arrows.", this);

    // shape_property_ =
    //     new rviz::EnumProperty("Shape", "Arrow", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));
    // shape_property_->addOption("Arrow", Arrow);
    // shape_property_->addOption("Axes", Axes);

    visual_enabled_property_ =
        new Property("Visual Enabled", true, "Whether to display the visual representation of the robot.", this,
                     SLOT(updateVisualVisible()));

    collision_enabled_property_ =
        new Property("Collision Enabled", false, "Whether to display the collision representation of the robot.", this,
                     SLOT(updateCollisionVisible()));

    alpha_property_ =
        new FloatProperty("Alpha", 1, "Amount of transparency to apply to the links.", this, SLOT(updateAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    robot_description_property_ = new StringProperty(
        "Robot Description", "robot_description", "Name of the parameter to search for to load the robot description.",
        this, SLOT(updateRobotDescription()));

    tf_prefix_property_ = new StringProperty(
        "TF Prefix", "", "Robot Model normally assumes the link name is the same as the tf frame name. "
                         " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
        this, SLOT(updateTfPrefix()));
}

RobotArrayDisplay::~RobotArrayDisplay()
{
    if (initialized())
    {
        scene_manager_->destroyManualObject(manual_object_);
    }
}

void RobotArrayDisplay::onInitialize()
{
    MFDClass::onInitialize();
    manual_object_ = scene_manager_->createManualObject();
    manual_object_->setDynamic(true);
    scene_node_->attachObject(manual_object_);

    robot_ = new Robot(scene_node_, context_, "Robot: " + getName().toStdString(), this);

    updateVisualVisible();
    updateCollisionVisible();
    updateAlpha();
}

void RobotArrayDisplay::updateAlpha()
{
    robot_->setAlpha(alpha_property_->getFloat());
    context_->queueRender();
}

void RobotArrayDisplay::updateRobotDescription()
{
    if (isEnabled())
    {
        load();
        context_->queueRender();
    }
}

void RobotArrayDisplay::updateVisualVisible()
{
    robot_->setVisualVisible(visual_enabled_property_->getValue().toBool());
    context_->queueRender();
}

void RobotArrayDisplay::updateCollisionVisible()
{
    robot_->setCollisionVisible(collision_enabled_property_->getValue().toBool());
    context_->queueRender();
}

void RobotArrayDisplay::updateTfPrefix()
{
    clearStatuses();
    context_->queueRender();
}

void RobotArrayDisplay::load()
{
    clearStatuses();

    std::string content;
    if (!update_nh_.getParam(robot_description_property_->getStdString(), content))
    {
        std::string loc;
        if (update_nh_.searchParam(robot_description_property_->getStdString(), loc))
        {
            update_nh_.getParam(loc, content);
        }
        else
        {
            clear();
            setStatus(StatusProperty::Error, "URDF", "Parameter [" + robot_description_property_->getString() +
                                                         "] does not exist, and was not found by searchParam()");
            return;
        }
    }

    if (content.empty())
    {
        clear();
        setStatus(StatusProperty::Error, "URDF", "URDF is empty");
        return;
    }

    if (content == robot_description_)
    {
        return;
    }

    robot_description_ = content;

    TiXmlDocument doc;
    doc.Parse(robot_description_.c_str());
    if (!doc.RootElement())
    {
        clear();
        setStatus(StatusProperty::Error, "URDF", "URDF failed XML parse");
        return;
    }

    urdf::Model descr;
    if (!descr.initXml(doc.RootElement()))
    {
        clear();
        setStatus(StatusProperty::Error, "URDF", "URDF failed Model parse");
        return;
    }

    setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
    robot_->load(descr);
    robot_->update(TFLinkUpdater(context_->getFrameManager(), boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
                                 tf_prefix_property_->getStdString()));
}

void RobotArrayDisplay::onEnable()
{
    load();
    robot_->setVisible(true);
}

void RobotArrayDisplay::onDisable()
{
    robot_->setVisible(false);
    clear();
}

// void RobotArrayDisplay::allocateCoords(int num)
// {
//     if (num > coords_objects_.size())
//     {
//         for (size_t i = coords_objects_.size(); i < num; i++)
//         {
//             Ogre::SceneNode* scene_node = scene_node_->createChildSceneNode();
//             rviz::Axes* axes = new rviz::Axes(scene_manager_, scene_node, axes_length_property_->getFloat(),
//                                               axes_radius_property_->getFloat());
//             coords_nodes_.push_back(scene_node);
//             coords_objects_.push_back(axes);
//         }
//     }
//     else if (num < coords_objects_.size())
//     {
//         for (int i = coords_objects_.size() - 1; num <= i; i--)
//         {
//             delete coords_objects_[i];
//             scene_manager_->destroySceneNode(coords_nodes_[i]);
//         }
//         coords_objects_.resize(num);
//         coords_nodes_.resize(num);
//     }
// }

bool validateFloats(const geometry_msgs::PoseArray& msg)
{
    return rviz::validateFloats(msg.poses);
}

void RobotArrayDisplay::processMessage(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if (!validateFloats(*msg))
    {
        setStatus(rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or "
                                                        "infs)");
        return;
    }

    if (msg->poses.empty())
    {
        setStatus(rviz::StatusProperty::Error, "Topic", "Message has no poses!");
        return;
    }

    setStatus(rviz::StatusProperty::Ok, "Topic", "Message contains " + QString::number(msg->poses.size()) + " poses");
    //                  "Message contains " + QString::fromStdString(std::to_string(msg->poses.size())) + " poses");

    manual_object_->clear();

    const auto& pose = *(msg->poses.cbegin());
    robot_->setPosition(toOgre(pose.position));
    robot_->setOrientation(toOgre(pose.orientation));

    // Ogre::Vector3 position;
    // Ogre::Quaternion orientation;
    // if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
    // {
    //     ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
    //               qPrintable(fixed_frame_));
    // }

    // pose_valid_ = true;
    // updateShapeVisibility();

    // scene_node_->setPosition(position);
    // scene_node_->setOrientation(orientation);

    // manual_object_->clear();

    // for (int i = 0; i < coords_nodes_.size(); i++)
    //     coords_nodes_[i]->setVisible(false);
    // Ogre::ColourValue color = color_property_->getOgreColor();
    // size_t num_poses = msg->poses.size();
    // manual_object_->estimateVertexCount(num_poses * 6);
    // manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    // for (size_t i = 0; i < num_poses; ++i)
    // {
    //     Ogre::Vector3 pos(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
    //     Ogre::Quaternion orient(msg->poses[i].orientation.w, msg->poses[i].orientation.x,
    //     msg->poses[i].orientation.y,
    //                             msg->poses[i].orientation.z);

    //     for (int i = 0; i < 6; ++i)
    //     {
    //         //                manual_object_->position(vertices[i]);
    //         manual_object_->colour(color);
    //     }
    // }
    manual_object_->end();

    context_->queueRender();
}

void RobotArrayDisplay::reset()
{
    MFDClass::reset();
    if (manual_object_)
    {
        manual_object_->clear();
    }
    // if (coords_objects_.size() > 0)
    // {
    //     allocateCoords(0);
    // }
}

void RobotArrayDisplay::clear()
{
    robot_->clear();
    clearStatuses();
    robot_description_.clear();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_array_plugin::RobotArrayDisplay, rviz::Display)
