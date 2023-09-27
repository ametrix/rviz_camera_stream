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

#include <rviz_camera_stream/video_publisher.hpp>

#include <rviz_common/load_resource.hpp>

#include <OgreRenderTexture.h>
#include <OgreRenderTarget.h>
#include <OgrePixelFormat.h>
#include <OgreImage.h>

#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace video_export
{

VideoPublisher::VideoPublisher(const rclcpp::Node::SharedPtr &node) :
  nh_(node),
  it_(node)
{
}

std::string VideoPublisher::getTopic()
{
  return pub_.getTopic();
}

sensor_msgs::msg::CameraInfo::SharedPtr VideoPublisher::cameraInfo() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return camera_info_;
}

bool VideoPublisher::isActive()
{
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ok = camera_info_ != nullptr && !camera_info_->header.frame_id.empty();
  }

  return !getTopic().empty() && ok;
}

void VideoPublisher::shutdown()
{
  if (!getTopic().empty()) {
    pub_.shutdown();
  }
}

void VideoPublisher::setCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr &camera_info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  camera_info_ = camera_info;
}

void VideoPublisher::advertise(std::string topic)
{
  pub_ = it_.advertiseCamera(topic, 1);
}

bool VideoPublisher::publishFrame(Ogre::RenderTexture * render_object, int encoding_option) {
    if (!isActive()) {
      return false;
    }

    // The suggested pixel format is most efficient, but other ones
    // can be used.
    sensor_msgs::msg::Image image;
    Ogre::PixelFormat pf = Ogre::PF_BYTE_RGB;
    switch (encoding_option)
    {
      case 0:
        pf = Ogre::PF_BYTE_RGB;
        image.encoding = sensor_msgs::image_encodings::RGB8;
        break;
      case 1:
        pf = Ogre::PF_BYTE_RGBA;
        image.encoding = sensor_msgs::image_encodings::RGBA8;
        break;
      case 2:
        pf = Ogre::PF_BYTE_BGR;
        image.encoding = sensor_msgs::image_encodings::BGR8;
        break;
      case 3:
        pf = Ogre::PF_BYTE_BGRA;
        image.encoding = sensor_msgs::image_encodings::BGRA8;
        break;
      case 4:
        pf = Ogre::PF_L8;
        image.encoding = sensor_msgs::image_encodings::MONO8;
        break;
      case 5:
        pf = Ogre::PF_L16;
        image.encoding = sensor_msgs::image_encodings::MONO16;
        break;
      default:
        RCLCPP_ERROR(nh_->get_logger(), "Invalid image encoding value specified");
        return false;
    }

    const uint32_t height = render_object->getHeight();
    const uint32_t width = render_object->getWidth();
    const uint32_t pixelsize = Ogre::PixelUtil::getNumElemBytes(pf);
    const uint32_t datasize = width * height * pixelsize;

    image.height = height;
    image.width = width;
    image.step = pixelsize * width;
    image.is_bigendian = (OGRE_ENDIAN == OGRE_ENDIAN_BIG);
    image.data.resize(datasize);

    Ogre::PixelBox pb(width, height, 1, pf, image.data.data());
    const Ogre::Box src(0, 0, width, height);
    render_object->copyContentsToMemory(src, pb, Ogre::RenderTarget::FB_AUTO);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      camera_info_->header.stamp = nh_->now();
      image.header = camera_info_->header;
      pub_.publish(image, *camera_info_);
    }

    return true;
}

}  // namespace video_export