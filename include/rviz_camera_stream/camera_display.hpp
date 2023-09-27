/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_CAMERA_STREAM_CAMERA_DISPLAY_HPP
#define RVIZ_CAMERA_STREAM_CAMERA_DISPLAY_HPP

#include <QObject>

#include <string>

#ifndef Q_MOC_RUN
#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rviz_default_plugins/displays/image/image_display.hpp>
#include <std_srvs/srv/trigger.hpp>
#endif

namespace Ogre
{
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;
}

namespace video_export
{
class  VideoPublisher;
}

namespace rviz_common {
namespace properties {
class EnumProperty;
class FloatProperty;
class IntProperty;
class RenderPanel;
class RosTopicProperty;
class DisplayGroupVisibilityProperty;
class ColorProperty;
class StringProperty;
}
}

namespace rviz
{

/**
 * \class CameraPub
 *
 */
class CameraPub: public rviz_common::Display, public Ogre::RenderTargetListener
{
  Q_OBJECT

public:
  static const QString BACKGROUND;
  static const QString OVERLAY;
  static const QString BOTH;

  CameraPub();
  virtual ~CameraPub();

protected:
  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  // Overrides from Ogre::RenderTargetListener
  virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
  virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void forceRender();
  void updateTopic();
  void updateQueueSize();
  void updateFrameRate();
  void updateBackgroundColor();
  void updateImageEncoding();
  void updateNearClipDistance();

private:
  static inline int count_ = 0;

  bool triggerCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void caminfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void subscribe();
  void unsubscribe();
  void clear();
  void updateStatus();

  bool force_render_ {false};
  bool trigger_activated_ {false};

  uint32_t vis_bit_{0};

  std::string camera_trigger_name_;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Time last_image_publication_time_;

  rviz_common::properties::RosTopicProperty* topic_property_{nullptr};
  rviz_common::properties::RosTopicProperty* camera_info_property_{nullptr};
  rviz_common::properties::DisplayGroupVisibilityProperty* visibility_property_{nullptr};
  rviz_common::properties::IntProperty* queue_size_property_{nullptr};

  rviz_common::properties::FloatProperty* frame_rate_property_{nullptr};
  rviz_common::properties::ColorProperty* background_color_property_{nullptr};
  rviz_common::properties::EnumProperty* image_encoding_property_{nullptr};
  rviz_common::properties::FloatProperty* near_clip_property_{nullptr};

  std::shared_ptr<video_export::VideoPublisher> video_publisher_;

  // render to texture
  // from http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Intermediate+Tutorial+7
  Ogre::SceneNode* camera_node_{nullptr};
  Ogre::Camera* camera_{nullptr};
  Ogre::TexturePtr rtt_texture_{nullptr};
  Ogre::RenderTexture* render_texture_{nullptr};
};

}  // namespace rviz

#endif  // RVIZ_CAMERA_STREAM_CAMERA_DISPLAY_HPP
