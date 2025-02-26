/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_camera_stream/camera_display.hpp"

#include <string>
#include <chrono>
#include <random>
#include <functional>
#include <iostream>

#include <rviz_common/bit_allocator.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/validate_floats.hpp>

#include <rviz_common/properties/display_group_visibility_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderTexture.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <rmw/types.h>

#include "rviz_camera_stream/video_publisher.hpp"

namespace rviz
{

const QString CameraPub::BACKGROUND("background");
const QString CameraPub::OVERLAY("overlay");
const QString CameraPub::BOTH("background and overlay");

bool validateFloats(const sensor_msgs::msg::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.d);
  valid = valid && rviz_common::validateFloats(msg.k);
  valid = valid && rviz_common::validateFloats(msg.r);
  valid = valid && rviz_common::validateFloats(msg.p);
  return valid;
}

CameraPub::CameraPub()
  : camera_trigger_name_("camera_trigger")
{
  std::string dataType = rosidl_generator_traits::data_type<sensor_msgs::msg::Image>();
  topic_property_ = new rviz_common::properties::RosTopicProperty("Image Topic", "",
      QString::fromStdString(dataType),
      "sensor_msgs::Image topic to publish to.", this, SLOT(updateTopic()));

  dataType = rosidl_generator_traits::data_type<sensor_msgs::msg::CameraInfo>();
  camera_info_property_ = new rviz_common::properties::RosTopicProperty("Camera Info Topic", "",
      QString::fromStdString(dataType),
      "sensor_msgs::CameraInfo topic to subscribe to.", this, SLOT(updateTopic()));

  queue_size_property_ = new rviz_common::properties::IntProperty( "Queue Size", 2,
      "Advanced: set the size of the incoming message queue.  Increasing this "
      "is useful if your incoming TF data is delayed significantly from your"
      " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT(updateQueueSize()));
  queue_size_property_->setMin(1);

  frame_rate_property_ = new rviz_common::properties::FloatProperty("Frame Rate", -1,
      "Sets target frame rate. Set to < 0 for maximum speed, set to 0 to stop, you can "
      "trigger single images with the /rviz_camera_trigger service.",
                                           this, SLOT(updateFrameRate()));
  frame_rate_property_->setMin(-1);

  background_color_property_ = new rviz_common::properties::ColorProperty("Background Color", Qt::black,
      "Sets background color, values from 0.0 to 1.0.",
                                           this, SLOT(updateBackgroundColor()));

  image_encoding_property_ = new rviz_common::properties::EnumProperty("Image Encoding", "rgb8",
      "Sets the image encoding", this, SLOT(updateImageEncoding()));
  image_encoding_property_->addOption("rgb8", 0);
  image_encoding_property_->addOption("rgba8", 1);
  image_encoding_property_->addOption("bgr8", 2);
  image_encoding_property_->addOption("bgra8", 3);
  image_encoding_property_->addOption("mono8", 4);
  image_encoding_property_->addOption("mono16", 5);

  near_clip_property_ = new rviz_common::properties::FloatProperty("Near Clip Distance", 0.01, "Set the near clip distance",
      this, SLOT(updateNearClipDistance()));
  near_clip_property_->setMin(0.01);
}

CameraPub::~CameraPub()
{
  if (initialized())
  {
    render_texture_->removeListener(this);

    unsubscribe();

    context_->visibilityBits()->freeBits(vis_bit_);
  }
}

bool CameraPub::triggerCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  res->success = video_publisher_->isActive();
  if (res->success) {
    trigger_activated_ = true;
    res->message = "New image will be published on: " + video_publisher_->getTopic();
  } else {
    res->message = "Image publisher not configured";
  }

  return true;
}

void CameraPub::onInitialize()
{
  rviz_common::Display::onInitialize();

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  if (!nodeAbstraction) {
    throw std::runtime_error("Error: nodeAbstraction is NULL!");
  }

  nh_ = nodeAbstraction->get_raw_node();
  last_image_publication_time_ = nh_->now();

  topic_property_->initialize(context_->getRosNodeAbstraction());
  camera_info_property_->initialize(context_->getRosNodeAbstraction());

  count_++;

  video_publisher_ = std::make_shared<video_export::VideoPublisher>(nh_);

  std::stringstream ss;
  ss << "RvizCameraPubCamera" << count_;

  camera_ = context_->getSceneManager()->createCamera(ss.str());
  camera_->setNearClipDistance(0.01f);
  camera_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  camera_node_->setPosition(0, 10, 15);
  camera_node_->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TransformSpace::TS_WORLD);
  camera_node_->attachObject(camera_);

  std::stringstream ss_tex;
  ss_tex << "RttTexInit" << count_;
  RCLCPP_INFO_STREAM(nh_->get_logger(), ss_tex.str());

  // render to texture
  rtt_texture_ = Ogre::TextureManager::getSingleton().createManual(
      ss_tex.str(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      640, 480,
      0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);
  render_texture_ = rtt_texture_->getBuffer()->getRenderTarget();
  render_texture_->addViewport(camera_);
  render_texture_->getViewport(0)->setClearEveryFrame(true);
  render_texture_->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
  render_texture_->getViewport(0)->setOverlaysEnabled(false);
  render_texture_->setAutoUpdated(false);
  render_texture_->setActive(false);
  render_texture_->addListener(this);

  // Thought this was optional but the plugin crashes without it
  vis_bit_ = context_->visibilityBits()->allocBit();
  render_texture_->getViewport(0)->setVisibilityMask(vis_bit_);

  visibility_property_ = new rviz_common::properties::DisplayGroupVisibilityProperty(
    vis_bit_, context_->getRootDisplayGroup(), this, "Visibility", true,
    "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon(rviz_common::loadPixmap("package://rviz/icons/visibility.svg", true));

  // this->addChild(visibility_property_, 0);

  trigger_service_ = nh_->create_service<std_srvs::srv::Trigger>(
    camera_trigger_name_, std::bind(&CameraPub::triggerCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void CameraPub::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
  context_->queueRender();
}

void CameraPub::preRenderTargetUpdate(const Ogre::RenderTargetEvent& /*evt*/)
{
  // set view flags on all displays
  visibility_property_->update();
}

void CameraPub::postRenderTargetUpdate(const Ogre::RenderTargetEvent& /*evt*/)
{
  // Publish the rendered window video stream
  const rclcpp::Time cur_time = nh_->now();
  rclcpp::Duration elapsed_duration = cur_time - last_image_publication_time_;
  const float frame_rate = frame_rate_property_->getFloat();
  bool time_is_up = (frame_rate > 0.0) && (elapsed_duration.seconds() > 1.0 / frame_rate);
  // We want frame rate to be unlimited if we enter zero or negative values for frame rate
  if (frame_rate < 0.0)
  {
    time_is_up = true;
  }
  if (!(trigger_activated_ || time_is_up))
  {
    return;
  }
  trigger_activated_ = false;
  last_image_publication_time_ = cur_time;
  render_texture_->getViewport(0)->setBackgroundColour(background_color_property_->getOgreColor());

  int encoding_option = image_encoding_property_->getOptionInt();
  video_publisher_->publishFrame(render_texture_, encoding_option);
}

void CameraPub::onEnable()
{
  subscribe();
  render_texture_->setActive(true);
}

void CameraPub::onDisable()
{
  render_texture_->setActive(false);
  unsubscribe();
  clear();
}

void CameraPub::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  std::string topic_name = topic_property_->getTopicStd();
  if (topic_name.empty())
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Output Topic", "No topic set");
    return;
  }

  // TODO: Add name validation: https://answers.ros.org/question/397554/ros2-topic-name-validation/
  //
  // std::string error;
  // if (!ros::names::validate(topic_name, error))
  // {
  //   setStatus(StatusProperty::Error, "Output Topic", QString(error.c_str()));
  //   return;
  // }

  std::string caminfo_topic = camera_info_property_->getTopicStd();
  if (caminfo_topic.empty())
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info", "No topic set");
    return;
  }

  // TODO: Add name validation: https://answers.ros.org/question/397554/ros2-topic-name-validation/
  //

  try
  {
    rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
    qosLatching.transient_local();
    qosLatching.reliable();
    caminfo_sub_ = nh_->create_subscription<sensor_msgs::msg::CameraInfo>(
      caminfo_topic, qosLatching, std::bind(&CameraPub::caminfoCallback, this, std::placeholders::_1));

    setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "OK");
  }
  catch (const rclcpp::exceptions::InvalidNodeNameError &e)
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info", QString("Error subscribing: ") + e.what());
  }

  video_publisher_->advertise(topic_name);
  setStatus(rviz_common::properties::StatusProperty::Ok, "Output Topic", "Topic set");
}

void CameraPub::unsubscribe()
{
  video_publisher_->shutdown();
  caminfo_sub_.reset();
}

void CameraPub::forceRender()
{
  force_render_ = true;
  context_->queueRender();
}

void CameraPub::updateQueueSize()
{
}

void CameraPub::updateFrameRate()
{
}

void CameraPub::updateNearClipDistance()
{
}


void CameraPub::updateBackgroundColor()
{
}

void CameraPub::updateImageEncoding()
{
}

void CameraPub::clear()
{
  force_render_ = true;
  context_->queueRender();

  // NOTE: Retain last camera info and don't clear it
  // video_publisher_->setCameraInfo(nullptr);

  std::string topic = "unknown";
  if (caminfo_sub_) {
    topic = caminfo_sub_->get_topic_name();
  }

  setStatus(rviz_common::properties::StatusProperty::Warn, "Camera Info",
            "No CameraInfo received on [" +
            QString::fromStdString(topic) +
            "].  Topic may not exist.");
  // setStatus(StatusProperty::Warn, "Camera Info", "No CameraInfo received");
  camera_node_->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void CameraPub::update(float /*wall_dt*/, float /*ros_dt*/)
{
  auto info = video_publisher_->cameraInfo();
  if (!info)
  {
    return;
  }

  if (!validateFloats(*info))
  {
    setStatus(rviz_common::properties::StatusProperty::Error,
      "Camera Info", "Contains invalid floating point values (nans or infs)");
    return;
  }

  // if we're in 'exact' time mode, only show image if the time is exactly right
  rclcpp::Time rviz_time = context_->getFrameManager()->getTime();
  if (context_->getFrameManager()->getSyncMode() == rviz_common::FrameManagerIface::SyncExact &&
      rviz_time != info->header.stamp)
  {
    std::ostringstream s;
    s << "Time-syncing active and no info at timestamp " << (rviz_time.nanoseconds() / 1e9) << ".";
    setStatus(rviz_common::properties::StatusProperty::Warn, "Time", s.str().c_str());
    return;
  }

  // TODO(lucasw) this will make the img vs. texture size code below unnecessary
  if ((info->width != render_texture_->getWidth()) ||
      (info->height != render_texture_->getHeight()))
  {
    std::stringstream ss_tex;
    ss_tex << "RttTex" << count_;
    RCLCPP_INFO_STREAM(nh_->get_logger(), ss_tex.str());

    rtt_texture_ = Ogre::TextureManager::getSingleton().createManual(
        ss_tex.str(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        info->width, info->height,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);
    render_texture_ = rtt_texture_->getBuffer()->getRenderTarget();
    render_texture_->addViewport(camera_);
    render_texture_->getViewport(0)->setClearEveryFrame(true);
    render_texture_->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
    render_texture_->getViewport(0)->setVisibilityMask(vis_bit_);

    render_texture_->getViewport(0)->setOverlaysEnabled(false);
    render_texture_->setAutoUpdated(false);
    render_texture_->setActive(false);
    render_texture_->addListener(this);
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  const bool success = context_->getFrameManager()->getTransform(
      info->header.frame_id, info->header.stamp, position, orientation);
  if (!success)
  {
    std::string error;
    const bool has_problems = context_->getFrameManager()->transformHasProblems(
        info->header.frame_id,
        info->header.stamp, error);
    if (has_problems)
    {
      setStatus(rviz_common::properties::StatusProperty::Error, "getTransform", error.c_str());
      return;
    }
  }

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  float img_width = info->width;
  float img_height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if (img_width == 0)
  {
    // ROS_DEBUG("Malformed CameraInfo on camera [%s], width = 0", qPrintable(getName()));
    img_width = 640;
  }

  if (img_height == 0)
  {
    // ROS_DEBUG("Malformed CameraInfo on camera [%s], height = 0", qPrintable(getName()));
    img_height = 480;
  }

  if (img_height == 0.0 || img_width == 0.0)
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info",
              "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return;
  }

  double fx = info->p[0];
  double fy = info->p[5];

  float win_width = render_texture_->getWidth();
  float win_height = render_texture_->getHeight();
  float zoom_x = 1.0;
  float zoom_y = zoom_x;

  // Preserve aspect ratio
  if (win_width != 0 && win_height != 0)
  {
    float img_aspect = (img_width / fx) / (img_height / fy);
    float win_aspect = win_width / win_height;

    if (img_aspect > win_aspect)
    {
      zoom_y = zoom_y / img_aspect * win_aspect;
    }
    else
    {
      zoom_x = zoom_x / win_aspect * img_aspect;
    }
  }

  // Add the camera's translation relative to the left camera (from P[3]);
  double tx = -1 * (info->p[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (info->p[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);

  if (!rviz_common::validateFloats(position))
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info",
        "CameraInfo/P resulted in an invalid position calculation (nans or infs)");
    return;
  }

  const float near_clip_distance = near_clip_property_->getFloat();

  camera_->setNearClipDistance(near_clip_distance);
  camera_node_->setPosition(position);
  camera_node_->setOrientation(orientation);

  // calculate the projection matrix
  double cx = info->p[2];
  double cy = info->p[6];

  double far_plane = 100;
  double near_plane = near_clip_distance;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

  proj_matrix[0][0] = 2.0 * fx / img_width * zoom_x;
  proj_matrix[1][1] = 2.0 * fy / img_height * zoom_y;

  proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width) * zoom_x;
  proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5) * zoom_y;

  proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

  proj_matrix[3][2] = -1;

  camera_->setCustomProjectionMatrix(true, proj_matrix);

  setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "OK");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Time", "ok");

  force_render_ = false;
  render_texture_->update();

#if 0
  static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif
}

void CameraPub::caminfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  video_publisher_->setCameraInfo(msg);
}

void CameraPub::fixedFrameChanged()
{
  // std::string targetFrame = fixed_frame_.toStdString();
  rviz_common::Display::fixedFrameChanged();
}

void CameraPub::reset()
{
  rviz_common::Display::reset();
  clear();
}

}  // namespace rviz


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz::CameraPub, rviz_common::Display)
