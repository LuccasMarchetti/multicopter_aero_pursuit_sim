#pragma once

#include "aero_pursuit/streaming/gcs_link.hpp"
#include "aero_pursuit/streaming/stream_config.hpp"
#include "aero_pursuit/streaming/video_encoder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

namespace aero_pursuit
{

/// Encodes an image topic (RGB, debug or colorized depth) and sends it to the
/// GCS (aero_pursuit_gcs) through a Unix socket. Runs in the container: with intra-process
/// communication the ros_gz_bridge image arrives with no copy or serialization.
/// Only subscribes to the topic and encodes while a viewer is connected.
class VideoStreamerNode : public rclcpp::Node
{
public:
  explicit VideoStreamerNode(const rclcpp::NodeOptions & options);
  ~VideoStreamerNode() override;

private:
  using Image = sensor_msgs::msg::Image;

  void onLine(std::string_view line);
  void applyPending();
  void onImage(const std::shared_ptr<const Image> & msg);
  void onEncoded(const streaming::EncodedFrame & frame);
  void sendText(bool error, const std::string & text);

  std::unique_ptr<streaming::VideoEncoder> encoder_;
  std::unique_ptr<streaming::GcsLink> link_;

  std::mutex pending_mutex_;
  streaming::StreamConfig pending_;
  bool dirty_ = false;

  streaming::StreamConfig cfg_;          ///< Applied (executor thread)
  std::string subscribed_topic_;
  rclcpp::Subscription<Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr apply_timer_;
  std::atomic<std::uint32_t> seq_{0};
  std::int64_t next_frame_ns_ = 0;   ///< Next instant accepted by the fps limit
};

}  // namespace aero_pursuit
