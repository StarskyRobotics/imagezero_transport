// *****************************************************************************
//
// Copyright (c) 2016-2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/make_shared.hpp>
#include <imagezero_ros/ros_support.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <imagezero/encoder.h>
#include <imagezero/decoder.h>
#include <memory>

using namespace cv;
using namespace std;

/*! \mainpage imagezero_ros
 *
 * This package contains convenience methods for converting back and forth
 * between sensor_msgs::Image messages and sensor_msgs::CompressedImage messages.
 *
 * The ImageZero algorithm currently only operates on 24-bit PPM data, so under
 * the hood this uses OpenCV to convert messages into an appropriate format
 * and then run it through the algorithms in the imagezero package.
 */

namespace enc = sensor_msgs::image_encodings;

namespace IZ
{
  sensor_msgs::CompressedImage compressImage(const sensor_msgs::Image& image)
  {
    ros::Time start = ros::Time::now();

    if (!encode_tables_initialized) {
      encode_tables_initialized = true;
      IZ::initEncodeTable();
    }

    // Compressed image message
    sensor_msgs::CompressedImage compressed;
    compressed.header = image.header;
    compressed.format = image.encoding;

    // Bit depth of image encoding
    int bitDepth = enc::bitDepth(image.encoding);

    // Update ros message format header
    compressed.format += "; iz compressed ";

    // Check input format
    if ((bitDepth == 8) || (bitDepth == 16))
    {

      IZ::Image<> pi;
      pi.setWidth(image.width);
      pi.setSamplesPerLine(3*image.width); // always have 3 channels as far as IZ is concerned

      if(enc::numChannels(image.encoding) == 3)
          pi.setHeight(image.height);
      else if(enc::numChannels(image.encoding) == 1)
          pi.setHeight(image.height / 3); // If we have 1 channel, pack the data down into 3

      std::vector<uint8_t> srcData = image.data;

      pi.setData(&srcData[0]);
      compressed.data.resize(pi.height() * pi.width() * 4 + 33);
      unsigned char *destEnd = IZ::encodeImage(pi, &compressed.data[0]);
      size_t size = destEnd - &compressed.data[0];
      compressed.data.resize(size);
      
    ros::Time iz_time = ros::Time::now();
    ROS_INFO_THROTTLE(1, "Took %.4fms to compress %lu bytes to %lu(%lu) bytes (%.1f%%)",
                       (iz_time - start).toSec()*1000,
                       image.data.size(),
                       compressed.data.size(), size,
                       100.0 * (double)compressed.data.size() / (double)image.data.size());

    ROS_INFO_THROTTLE(1, "sz: %dx%d enc: %s chan: %d color: %d", pi.width(), pi.height(), image.encoding.c_str(), enc::numChannels(image.encoding), enc::isColor(image.encoding));

      return compressed;
    }
    else
    {
      ROS_ERROR(
          "Compressed Image Transport - ImageZero compression requires 8/16-bit encoded color format (input format is: %s)",
          image.encoding.c_str());
    }

    return sensor_msgs::CompressedImage();
  }

  sensor_msgs::Image decompressImage(const sensor_msgs::CompressedImageConstPtr& compressed)
  {
    if (!decode_tables_initialized){
      decode_tables_initialized = true;
      IZ::initDecodeTable();
    }

    IZ::Image<> pi;
    sensor_msgs::Image msg;
    IZ::decodeImageSize(pi, &compressed->data[0]);
    const unsigned int dataSize = pi.width() * pi.height() * 3;
    msg.data.resize(dataSize);
    pi.setData(&msg.data[0]);
    IZ::decodeImage(pi, &compressed->data[0]);

    const size_t split_pos = compressed->format.find(';');
    std::string image_encoding = compressed->format.substr(0, split_pos);
    msg.width = pi.width();
    msg.height = pi.height() * 3 / enc::numChannels(image_encoding); // if only 1 channel, we 3x the size, otherwise it's using actual size
    msg.encoding = image_encoding;
    msg.step = msg.width * enc::numChannels(image_encoding);

    ROS_INFO_THROTTLE(1, "sz: %dx%d enc: %s step: %d chan: %d color: %d", msg.width, msg.height, msg.encoding.c_str(), msg.step, enc::numChannels(msg.encoding), enc::isColor(msg.encoding));

    return msg;
  }
}
