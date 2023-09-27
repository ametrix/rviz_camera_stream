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

VideoPublisher::VideoPublisher() :
  nh_(rclcpp::Node::make_shared("video_publisher", rclcpp::NodeOptions())),
  it_(std::make_shared<image_transport::ImageTransport>(nh_))
{
}

std::string VideoPublisher::get_topic()
{
  return pub_.getTopic();
}

bool VideoPublisher::is_active()
{
  return !get_topic().empty();
}

void VideoPublisher::setNodehandle(const rclcpp::Node::SharedPtr& nh)
{
  shutdown();
  nh_ = nh;
  it_ = std::make_shared<image_transport::ImageTransport>(nh_);
}

void VideoPublisher::shutdown()
{
  if (get_topic() != "") {
    pub_.shutdown();
  }
}

void VideoPublisher::advertise(std::string topic)
{
  pub_ = it_->advertiseCamera(topic, 1);
}

  // bool publishFrame(Ogre::RenderWindow * render_object, const std::string frame_id)
bool VideoPublisher::publishFrame(Ogre::RenderTexture * render_object, const std::string frame_id, int encoding_option) {
    if (get_topic().empty()) {
      return false;
    }

    if (frame_id.empty()) {
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

    image.header.stamp = nh_->now();
    image.header.frame_id = frame_id;
    image.height = height;
    image.width = width;
    image.step = pixelsize * width;
    image.is_bigendian = (OGRE_ENDIAN == OGRE_ENDIAN_BIG);
    image.data.resize(datasize);
    camera_info_.header = image.header;

    Ogre::PixelBox pb(width, height, 1, pf, image.data.data());
    render_object->copyContentsToMemory(pb, Ogre::RenderTarget::FB_AUTO);

    pub_.publish(image, camera_info_);
    return true;
}



}  // namespace video_export