/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Giacomo Franchini
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FUSE_VIZ__POSE_STAMPED_VISUAL_HPP_
#define FUSE_VIZ__POSE_STAMPED_VISUAL_HPP_

#include <tf2/LinearMath/Transform.h>
#include <Ogre.h>

#include <memory>

#include <fuse_viz/conversions.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/objects/object.hpp>
#include <rviz_rendering/objects/shape.hpp>



namespace Ogre
{

class SceneManager;
class SceneNode;
class Any;

}  // namespace Ogre

namespace fuse_viz
{

using rviz_rendering::MovableText;

/**
 * @class PoseStampedVisual
 *
 * @brief 2D or 3D pose variable visual consisting on:
 * 1. A sphere representing the 2D/3D position.
 * 2. An axes representing the 2D/3D pose (position + orientation).
 * 3. A text with the variable type and UUID.
 */
template <typename PositionT, typename OrientationT>
class PoseStampedVisual : public rviz_rendering::Object
{
private:
  /**
   * @brief Private Constructor
   *
   * PoseStampedVisual can only be constructed by friend class
   * Pose2DStampedProperty or Pose3DStampedProperty.
   *
   * @param[in] scene_manager The scene manager to use to construct any necessary objects
   * @param[in] parent_object A rviz object that this constraint will be attached.
   * @param[in] position PositionVariable position.
   * @param[in] orientation OrientationVariable orientation.
   * @param[in] visible Initial visibility.
   */
  PoseStampedVisual(
    Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node,
    const PositionT & position,
    const OrientationT & orientation, const bool visible = true) 
    : Object(scene_manager), root_node_(parent_node->createChildSceneNode()), visible_(visible)
  {
    // Create sphere:
    sphere_node_ = root_node_->createChildSceneNode();
    sphere_ = std::make_shared<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, scene_manager_,
      sphere_node_);
    setSphereColor(1.0, 0.0, 0.0, 1.0);

    // Create axes:
    axes_node_ = root_node_->createChildSceneNode();
    axes_ = std::make_shared<rviz_rendering::Axes>(scene_manager_, axes_node_, 10.0, 1.0);

    // Create text:
    const auto caption = position.type() + "::" + fuse_core::uuid::to_string(position.uuid()) + '\n' +
      orientation.type() + "::" + fuse_core::uuid::to_string(orientation.uuid());
    text_ = new MovableText(caption);
    text_->setCaption(caption);
    text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);
    text_->showOnTop();

    text_node_ = root_node_->createChildSceneNode();
    text_node_->attachObject(text_);

    // Set pose for all objects:
    setPoseStamped(position, orientation);

    // Set the initial visibility:
    // The root node is always visible. The visibility will be updated on its childs.
    root_node_->setVisible(true);
    setVisible(visible_);
  }

public:
  ~PoseStampedVisual() override
  {
    delete text_;
    scene_manager_->destroySceneNode(sphere_node_);
    scene_manager_->destroySceneNode(axes_node_);
    scene_manager_->destroySceneNode(text_node_);
    scene_manager_->destroySceneNode(root_node_);
  };
  /**
   * @brief Set pose stamped.
   * @param[in] position    PositionT stamped variable.
   * @param[in] orientation OrientationT stamped variable.
   */
  void setPoseStamped(
    const PositionT & position,
    const OrientationT & orientation)
  {
    setPoseStamped(toOgre(position), toOgre(orientation));
  }
  /**
   * @brief Get the root scene node of this variable visual.
   * @return the root scene node of this variable visual.
   */
  Ogre::SceneNode * getSceneNode()
  {
    return root_node_;
  }

  /**
   * @brief Sets user data on all ogre objects we own
   */
  void setUserData(const Ogre::Any & data) override
  {
    axes_->setUserData(data);
    sphere_->setUserData(data);
  }

  void setSphereColor(const float r, const float g, const float b, const float a)
  {
    sphere_->setColor(r, g, b, a);
  }

  void setAxesAlpha(const float alpha)
  {
    static const auto & default_x_color_ = axes_->getDefaultXColor();
    static const auto & default_y_color_ = axes_->getDefaultYColor();
    static const auto & default_z_color_ = axes_->getDefaultZColor();

    axes_->setXColor(
      Ogre::ColourValue(
        default_x_color_.r, default_x_color_.g, default_x_color_.b,
        alpha));                                                                                               // NOLINT
    axes_->setYColor(
      Ogre::ColourValue(
        default_y_color_.r, default_y_color_.g, default_y_color_.b,
        alpha));                                                                                               // NOLINT
    axes_->setZColor(
      Ogre::ColourValue(
        default_z_color_.r, default_z_color_.g, default_z_color_.b,
        alpha));                                                                                               // NOLINT
  }

  void setScale(const Ogre::Vector3 & scale) override
  {
    sphere_->setScale(scale);
    axes_->setScale(scale);
  }

  void setTextScale(const Ogre::Vector3 & scale)
  {
    text_node_->setScale(scale);
  }

  void setTextVisible(const bool visible)
  {
    text_->setVisible(visible);
  }

  /**
   * @brief Sets visibility of this constraint
   *
   * Convenience method that sets visibility
   */
  void setVisible(bool visible)
  {
    sphere_node_->setVisible(visible);
    axes_node_->setVisible(visible);
  }

  /**
   * @brief Sets position of the frame this constraint is attached
   */
  void setPosition(const Ogre::Vector3 & position) override
  {
    root_node_->setPosition(position);
  }

  /**
   * @brief Sets orientation of the frame this constraint is attached
   */
  void setOrientation(const Ogre::Quaternion & orientation) override
  {
    root_node_->setOrientation(orientation);
  }

private:
  void setPoseStamped(const Ogre::Vector3 & position, const Ogre::Quaternion & orientation)
  {
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
    sphere_->setPosition(position);
    text_node_->setPosition(position);
  }

  Ogre::SceneNode * root_node_ = nullptr;
  Ogre::SceneNode * sphere_node_ = nullptr;
  Ogre::SceneNode * axes_node_ = nullptr;
  Ogre::SceneNode * text_node_ = nullptr;

  bool visible_;

  std::shared_ptr<rviz_rendering::Axes> axes_;
  std::shared_ptr<rviz_rendering::Shape> sphere_;
  rviz_rendering::MovableText * text_;

private:
  // Hide Object methods we don't want to expose
  // NOTE: Apparently we still need to define them...
  void setColor(float, float, float, float) override {}
  const Ogre::Vector3 & getPosition() override;
  const Ogre::Quaternion & getOrientation() override;

  // Make Pose2DStampedProperty and Pose3DStampedProperty friend class so it create PoseStampedVisual objects
  friend class Pose2DStampedProperty;
  friend class Pose3DStampedProperty;

};

}  // namespace fuse_viz

#endif  // FUSE_VIZ__POSE_STAMPED_VISUAL_HPP_
