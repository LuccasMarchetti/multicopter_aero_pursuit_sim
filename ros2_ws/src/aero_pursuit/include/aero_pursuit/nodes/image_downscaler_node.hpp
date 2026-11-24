#pragma once

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace aero_pursuit
{

/// Prepares the camera streams for a pursuer that runs on another computer (Raspberry Pi,
/// docs/RASPBERRY.md). Runs on the simulator side, in the same container as the bridge
/// (intra-process: the full images are never serialized).
///
/// Raw over the network does not work: RGB 1920x1080 at 30 Hz is ~1.5 Gbit/s and depth 640x480
/// float32 another ~0.3 Gbit/s; even reduced, multi-megabyte samples keep Fast DDS's receive
/// thread busy reassembling fragments and delay the small control messages (odometry, attitude)
/// that share it. Here:
///  - RGB: resized by `scale` and sent as JPEG (`jpeg_quality`, ~0.1 MB/frame) on
///    `<output_topic>/compressed`; `jpeg_quality: 0` sends raw on `output_topic`;
///  - depth: resized by `depth_scale` (nearest neighbour: no mixing of depths across edges) and,
///    with `depth_png_level` >= 0, sent as lossless PNG of 16-bit millimetres on
///    `<depth_output_topic>/compressed` (~5x less than float32; -1 sends raw);
///  - `scale: 1.0` and `depth_scale: 1.0` keep the full resolution (JPEG 1080p ~0.2 MB/frame).
///  - CameraInfo of both scaled with the images (fx, fy, cx, cy, P), so the bearings and the
///    RGB-to-depth mapping computed on the pursuer are the same as at full resolution.
class ImageDownscalerNode : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  explicit ImageDownscalerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// CameraInfo of the image resized by `scale` (pinhole model; distortion unchanged).
  static CameraInfo scaled(const CameraInfo & info, double scale);

private:
  void onImage(Image::UniquePtr msg);
  void onDepth(Image::UniquePtr msg);

  double scale_ = 0.5;
  double depth_scale_ = 0.5;
  int jpeg_quality_ = 80;
  int depth_png_level_ = 1;  ///< zlib level of the depth PNG (-1: raw)
  double min_period_ = 0.0;  ///< 1 / max_rate_hz (0: every frame)
  rclcpp::Time last_out_{0, 0, RCL_ROS_TIME};
  cv::Mat bgr_, resized_, depth_resized_, depth_buffer_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_, sub_depth_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_, sub_depth_info_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_, pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_, pub_depth_compressed_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_info_, pub_depth_info_;
};

}  // namespace aero_pursuit
