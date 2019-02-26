#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <tinyxml.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/ros_topic_property.h>
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
    using std::isfinite;
    Ogre::Vector3 result;
    result.x = position.x;
    result.y = position.y;
    result.z = position.z;
    if (isfinite(result.x) and isfinite(result.y) and isfinite(result.z))
    {
        return result;
    }
    else
    {
        ROS_ERROR_STREAM(result << " vs " << position);
        return Ogre::Vector3(0.0, 0.0, 0.0);
    }
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

RobotArrayDisplay::RobotArrayDisplay() : rviz::Display(), has_new_transforms_(false), time_since_last_transform_(0.0f)
{
    visual_enabled_property_ =
        new Property("Visual Enabled", true, "Whether to display the visual representation of the robot.", this,
                     SLOT(updateVisualVisible()));

    collision_enabled_property_ =
        new Property("Collision Enabled", false, "Whether to display the collision representation of the robot.", this,
                     SLOT(updateCollisionVisible()));

    update_rate_property_ =
        new FloatProperty("Update Interval", 0, "Interval at which to update the links, in seconds. "
                                                " 0 means to update every update cycle.",
                          this);
    update_rate_property_->setMin(0);

    alpha_property_ =
        new FloatProperty("Alpha", 1, "Amount of transparency to apply to the links.", this, SLOT(updateAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    robot_description_property_ = new StringProperty(
        "Robot Description", "robot_description", "Name of the parameter to search for to load the robot description.",
        this, SLOT(updateRobotDescription()));

    pose_array_topic_property_ = new rviz::RosTopicProperty(
        "PoseArray Topic", "/pose_array", ros::message_traits::datatype<geometry_msgs::PoseArray>(),
        "The topic on which a new trajectory as a set of poses is received", this, SLOT(changedTopic()), this);

    sample_rate_property_ = new FloatProperty("Sample rate", 1, "Sample rate [0.0, 1.0]. Determines how often a "
                                                                "pose should be displayed",
                                              this, SLOT(updateSampling()));
    sample_rate_property_->setMin(0.0);
    sample_rate_property_->setMax(1.0);

    tf_prefix_property_ = new StringProperty(
        "TF Prefix", "", "Robot Model normally assumes the link name is the same as the tf frame name. "
                         " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
        this, SLOT(updateTfPrefix()));
}

RobotArrayDisplay::~RobotArrayDisplay()
{
}

void RobotArrayDisplay::onInitialize()
{
    robots_.resize(3);
    robots_[0].reset(new rviz::Robot(scene_node_, context_, "Robot: " + getName().toStdString(), this));
    robots_[1].reset(new rviz::Robot(scene_node_, context_, "Robot2: " + getName().toStdString(), this));
    robots_[2].reset(new rviz::Robot(scene_node_, context_, "Robot3: " + getName().toStdString(), this));

    updateVisualVisible();
    updateCollisionVisible();
    updateAlpha();
}

void RobotArrayDisplay::updateAlpha()
{
    for (const auto& robot : robots_)
    {
        robot->setAlpha(alpha_property_->getFloat());
    }
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

void RobotArrayDisplay::changedTopic()
{
    pose_array_sub_.shutdown();
    // post-pone subscription if robot_state_ is not yet defined, i.e. onRobotModelLoaded() not yet called
    if (!pose_array_topic_property_->getStdString().empty())
    {
        pose_array_sub_ = update_nh_.subscribe(pose_array_topic_property_->getStdString(), 2,
                                               &RobotArrayDisplay::processMessage, this);
    }
}

void RobotArrayDisplay::updateSampling()
{
    ROS_INFO_STREAM("Updated sampling to " << sample_rate_property_->getFloat());
}

void RobotArrayDisplay::updateVisualVisible()
{
    for (const auto& robot : robots_)
    {
        robot->setVisualVisible(visual_enabled_property_->getValue().toBool());
    }
    context_->queueRender();
}

void RobotArrayDisplay::updateCollisionVisible()
{
    for (const auto& robot : robots_)
    {
        robot->setCollisionVisible(collision_enabled_property_->getValue().toBool());
    }
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

    if (!urdf_description_.initXml(doc.RootElement()))
    {
        clear();
        setStatus(StatusProperty::Error, "URDF", "URDF failed Model parse");
        return;
    }

    setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
    for (const auto& robot : robots_)
    {
        robot->load(urdf_description_);
    }
}

void RobotArrayDisplay::onEnable()
{
    load();
    for (const auto& robot : robots_)
    {
        robot->setVisible(true);
    }
}

void RobotArrayDisplay::onDisable()
{
    for (const auto& robot : robots_)
    {
        robot->setVisible(false);
    }
    clear();
}

void RobotArrayDisplay::update(float wall_dt, float ros_dt)
{
    time_since_last_transform_ += wall_dt;
    float rate = update_rate_property_->getFloat();
    bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

    if (last_received_msg_)
    {
        if ((not last_received_msg_->poses.empty()) and (has_new_transforms_ or update))
        {
            for (const auto& robot : robots_)
            {
                robot->update(TFLinkUpdater(context_->getFrameManager(),
                                            boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
                                            tf_prefix_property_->getStdString()));
            }

            for (size_t i = 0; i < robots_.size(); ++i)
            {
                const auto& pose = last_received_msg_->poses[(i * 5) + 1];

                robots_[i]->setPosition(toOgre(pose.position));
                robots_[i]->setOrientation(toOgre(pose.orientation));
            }
            context_->queueRender();

            has_new_transforms_ = false;
            time_since_last_transform_ = 0.0f;
        }
    }
}

void RobotArrayDisplay::fixedFrameChanged()
{
    has_new_transforms_ = true;
}

void RobotArrayDisplay::clear()
{
    for (const auto& robot : robots_)
    {
        robot->clear();
    }
    clearStatuses();
    robot_description_.clear();
}

void RobotArrayDisplay::reset()
{
    Display::reset();
    has_new_transforms_ = true;
}

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

    setStatus(rviz::StatusProperty::Ok, "Topic", "Message pose count " + QString::number(msg->poses.size()));
    last_received_msg_ = msg;
}

}  // namespace robot_array_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_array_plugin::RobotArrayDisplay, rviz::Display)
