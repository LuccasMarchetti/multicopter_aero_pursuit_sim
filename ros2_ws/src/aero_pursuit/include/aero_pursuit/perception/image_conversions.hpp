#pragma once

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

// Lightweight sensor_msgs <-> cv::Mat conversions, replacing cv_bridge.
// Avoids unnecessary copies and the conflict between the system OpenCV (cv_bridge)
// and the local OpenCV 4.9 (TrackerVit) in the same process.
namespace aero_pursuit::perception
{

/// Zero-copy view over the message data (valid while the message exists).
/// Returns an empty Mat for unsupported encodings.
cv::Mat wrapImage(const sensor_msgs::msg::Image & msg);

/// Converts to BGR8 into `out`, reusing the buffer between calls.
bool toBgr(const sensor_msgs::msg::Image & msg, cv::Mat & out);

/// Depth in meters (CV_32FC1). No copy for 32FC1 (view over the message);
/// 16UC1 (mm) is converted into `conversion_buffer` (always with its own memory).
/// Returns an empty Mat for unsupported encodings.
cv::Mat toDepthMeters(const sensor_msgs::msg::Image & msg, cv::Mat & conversion_buffer);

void toImageMsg(
  const cv::Mat & bgr, const std_msgs::msg::Header & header, sensor_msgs::msg::Image & out);

/// Decodes a CompressedImage (JPEG/PNG) into BGR8 `out` (reusing its buffer).
bool fromCompressedMsg(const sensor_msgs::msg::CompressedImage & msg, cv::Mat & out);

bool toCompressedMsg(
  const cv::Mat & bgr, const std_msgs::msg::Header & header, int jpeg_quality,
  sensor_msgs::msg::CompressedImage & out);

/// Depth (32FC1 m or 16UC1 mm) as lossless PNG of 16UC1 millimetres (format "16UC1; png").
/// NaN -> 0 (invalid); +inf and anything beyond 65.534 m -> 65535 (beyond any range).
bool toCompressedDepthMsg(
  const sensor_msgs::msg::Image & depth, int png_level, cv::Mat & buffer,
  sensor_msgs::msg::CompressedImage & out);

/// Inverse of toCompressedDepthMsg: 16UC1 (mm) Image in `out`.
bool fromCompressedDepthMsg(
  const sensor_msgs::msg::CompressedImage & msg, sensor_msgs::msg::Image & out);

}  // namespace aero_pursuit::perception
